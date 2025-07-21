package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.ClipSampleCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.IntakeSampleChamberCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.LoadClipCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.PickupClipsCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.ScoreOnChamber;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.StowOuttakeSlides;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Sample;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class RedSpec extends OpMode {

    PinpointDrive drive;
    TelemetryPacket tele;
    SequentialAction path1, path2;
    boolean path1End, path2End, initialScan;
    RobotHardware robot = RobotHardware.getInstance();

    Sample targetedSample;

    private boolean firstRun;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        drive = new PinpointDrive(hardwareMap, Poses.RedSpec.start);
        tele = new TelemetryPacket();
        robot.init(hardwareMap);

        robot.limelight.start();

        targetedSample = new Sample(0,0,0,0);

        robot.clipMech.setRightClipHolderTurns(0);

        robot.limelightClass.targetSampleColor(1);
        robot.limelightClass.setSortingDirection(Limelight.SortingDirection.LEFT);

        firstRun = true;

        initialScan = true;

        path1End = false;

        path1 = createPath1();
        path2 = intakingLoop();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.autoPeriodic();

        if (firstRun) {

            if (robot.outtake.getTurns() != 0) {
                robot.outtake.setTurns(0);
            }

            if (robot.clipMech.getRightClipHolderTurns() != 0) {
                robot.clipMech.setRightClipHolderTurns(0);
            }

            firstRun = false;
        }

        telemetry.addData("Intake IK Turret Angle", Double.isNaN(IntakeInverseKinematics.turretAngle));
        telemetry.addData("Intake IK Slide Extension", IntakeInverseKinematics.slideExtension);
        telemetry.addData("Slide Extension Inches", IntakeInverseKinematics.slideExtensionInches);
        telemetry.addData("Left Clip Magazine Turns: ", robot.clipMech.getLeftClipHolderTurns());
        telemetry.addData("Right Clip Magazine Turns: ", robot.clipMech.getRightClipHolderTurns());
        //telemetry.addData("Outtake Motor Current: ", robot.outtakeMotorOne.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("Samples: ", Arrays.toString(robot.limel[ightClass.getSampleLocations()));
        telemetry.addData("Samples Visible: ", robot.limelightClass.getSamples().size());
        telemetry.addData("Targeted Sample X: ", targetedSample.x);


        if (!Globals.CLIP_LOADED && !Globals.SAMPLE_LOADED && robot.outtake.getOuttakeState() == Outtake.OuttakeState.STOWED && robot.clipMech.getClipMechState() == ClipMech.ClipMechState.STOWED) {
            CommandScheduler.getInstance().schedule(new LoadClipCommand());
            robot.clipMech.setClipMechState(ClipMech.ClipMechState.CLIPPING);
        }


        path1.run(tele);

        if (path1End) {
            if (initialScan) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.limelightClass.setTargetedSampleIndex(0)),
                                new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                new WaitCommand(100),
                                new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                new WaitCommand(100),
                                new InstantCommand(() -> targetedSample = robot.limelightClass.getTargetedSample()),
                                new InstantCommand(() -> IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r))
                        )
                );

//                targetedSample = robot.limelightClass.getTargetedSample();
//                IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r);

                path2 = intakingLoop();

                initialScan = false;
            }

            path2.run(tele);
        }

        if (path2End) {

            if (robot.intake.isSample()) {
                path2 = clipping();
            } else {
//                if (robot.limelightClass.getTargetedSampleIndex() == 0 || robot.limelightClass.getTargetedSampleIndex()+1 > robot.limelightClass.getSamples().size()-1) {
//                    CommandScheduler.getInstance().schedule(
//                            new SequentialCommandGroup(
//                                    new InstantCommand(() -> robot.limelightClass.setTargetedSampleIndex(0)),
//                                    new InstantCommand(() -> robot.limelightClass.refreshSamples()),
//                                    new WaitCommand(100),
//                                    new InstantCommand(() -> robot.limelightClass.refreshSamples()),
//                                    new WaitCommand(100)
//                            )
//                    );
//                } else {
//                    CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.limelightClass.targetNextSample()));
//                }
//
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> targetedSample = robot.limelightClass.getTargetedSample()),
//                                new InstantCommand(() -> IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r))
//                        )
//                );

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> robot.limelightClass.setTargetedSampleIndex(0)),
                                                new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                                new WaitCommand(100),
                                                new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                                new WaitCommand(100)

                                        ),
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> robot.limelightClass.targetNextSample()),
                                                new WaitCommand(200)
                                        ),
                                        () -> robot.limelightClass.getTargetedSampleIndex() == 0 || robot.limelightClass.getTargetedSampleIndex()+1 > robot.limelightClass.getSamples().size()-1
                                ),
                                new InstantCommand(() -> targetedSample = robot.limelightClass.getTargetedSample()),
                                new InstantCommand(() -> IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r))
                        )
                );

                path2 = intakingLoop();
            }

            path2End = false;
        }
    }

    public SequentialAction createPath1() {
        return new SequentialAction(
                goToSub(), endPath1()
        );
    }

    public SequentialAction endPath1() {
        return new SequentialAction(
                telemetryPacket -> {
                    path1End = true;
                    return false;
                }
        );
    }

    public SequentialAction intakingLoop() {
        return new SequentialAction(
                intakeAgain(), endPath2()
        );
    }

    public SequentialAction clipping() {
        return new SequentialAction(
                backUpClipAndScore(), endPath2()
        );
    }

    public SequentialAction endPath2() {
        return new SequentialAction(
                telemetryPacket -> {
                    path2End = true;
                    return false;
                }
        );
    }

    public SequentialAction scorePreload() {
        return new SequentialAction(

                new ParallelAction(
                        drive.actionBuilder(Poses.RedSpec.start)
                                .splineToLinearHeading(Poses.RedSpec.chamberPreload, Math.toRadians(0))
                                .build(),

                        telemetryPacket -> {
                            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                                    new OuttakeArmCommand(RobotConstants.Outtake.armPreload),
                                    new WaitCommand(200),
                                    new IntakeStateCommand(Intake.IntakeState.INTAKING_CHAMBER_1),
                                    new IntakeSampleChamberCommand()
                            ));
                            return false;
                        }
                ),

                new SleepAction(0.3),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                            new ScoreOnChamber(),
                            new WaitCommand(300),
                            new StowOuttakeSlides()
                    ));
                    return false;
                },

                new SleepAction(3.0),

                drive.actionBuilder(drive.pinpoint.getPositionRR())
                        .lineToX(Poses.RedSpec.chamberBackUp.position.x)
                        .build(),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                            new IntakeStateCommand(Intake.IntakeState.INTAKING_CHAMBER_3),
                            new IntakeSampleChamberCommand()
                    ));
                    return false;
                }

        );
    }

    public SequentialAction loadClips() {
        return new SequentialAction(
                drive.actionBuilder(drive.pinpoint.getPositionRR())
                        .splineToLinearHeading(Poses.RedSpec.pickupClipsFar, Math.toRadians(90))
                        .build(),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                    return false;
                },

                new SleepAction(0.5),

                drive.actionBuilder(Poses.RedSpec.pickupClipsFar)
                        .lineToX(Poses.RedSpec.pickupClipsClose.position.x)
                        .build(),

                new SleepAction(1.5),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                    return false;
                },

                new SleepAction(0.7),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                    return false;
                },

                new SleepAction(1),

                drive.actionBuilder(Poses.RedSpec.pickupClipsClose)
                        .lineToX(Poses.RedSpec.pickupClipsFar.position.x)
                        .build(),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                    return false;
                }
        );
    }

    public SequentialAction pickupSpikeMark() {
        return new SequentialAction(

                new SleepAction(0.5),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                new WaitCommand(300),
                                new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                new WaitCommand(300),
                                new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                new WaitCommand(500),
                                new InstantCommand(() -> targetedSample = robot.limelightClass.getTargetedSample()),
                                new InstantCommand(() -> IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r))
                            )
                    );
                    return false;
                },

                new SleepAction(1.0),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                new IntakeSampleCommand()
                            )
                    );
                    return false;
                }
        );
    }

    public SequentialAction goToSub() {
        return new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(Poses.RedSpec.start)
                                .splineToLinearHeading(Poses.RedSpec.intaking, Math.toRadians(0))
                                .build(),

                        telemetryPacket -> {
                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new IntakeStateCommand(Intake.IntakeState.INTAKING_CHAMBER_1),
                                            new IntakeSampleChamberCommand(),
                                            new WaitCommand(300)
                                    )
                            );
                            return false;
                        }
                )
        );
    }

    public SequentialAction backUpClipAndScore() {
        return new SequentialAction(
                drive.actionBuilder(Poses.RedSpec.intaking)
                        .lineToX(Poses.RedSpec.clippingBackUp.position.x)
                        .build(),

                new SleepAction(0.5),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new IntakeSampleChamberCommand());
                    return false;
                },

                new SleepAction(0.8),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new TransferSampleCommand(),
                                    new ClipSampleCommand()
                            )
                    );
                    return false;
                },

                new SleepAction(8),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                            new OuttakeArmCommand(RobotConstants.Outtake.armPreload),
                            new IntakeStateCommand(Intake.IntakeState.INTAKING_CHAMBER_1),
                            new IntakeSampleChamberCommand()
                    ));
                    return false;
                },

                new SleepAction(0.5),

                drive.actionBuilder(Poses.RedSpec.clippingBackUp)
                        .lineToX(Poses.RedSpec.intaking2.position.x)
                        .build(),

                new SleepAction(0.3),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                            new ScoreOnChamber(),
                            new WaitCommand(300),
                            new StowOuttakeSlides()
                    ));
                    return false;
                },

                new SleepAction(2.0)

        );
    }

    public SequentialAction intakeAgain() {
        return new SequentialAction(

                new SleepAction(0.5),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new IntakeSampleChamberCommand());
                    return false;
                },

                new SleepAction(3.25)
        );
    }

    public SequentialAction refreshSamples() {
        return new SequentialAction(
                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new ConditionalCommand(
                                            new SequentialCommandGroup(
                                                    new InstantCommand(() -> robot.limelightClass.setTargetedSampleIndex(0)),
                                                    new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                                    new WaitCommand(100),
                                                    new InstantCommand(() -> robot.limelightClass.refreshSamples()),
                                                    new WaitCommand(100)

                                            ),
                                            new SequentialCommandGroup(
                                                    new InstantCommand(() -> robot.limelightClass.targetNextSample()),
                                                    new WaitCommand(200)
                                            ),
                                            () -> robot.limelightClass.getTargetedSampleIndex() == 0 || robot.limelightClass.getTargetedSampleIndex()+1 > robot.limelightClass.getSamples().size()-1
                                    ),
                                    new InstantCommand(() -> targetedSample = robot.limelightClass.getTargetedSample()),
                                    new InstantCommand(() -> IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r))
                            )
                    );
                    return false;
                }
        );
    }
}
