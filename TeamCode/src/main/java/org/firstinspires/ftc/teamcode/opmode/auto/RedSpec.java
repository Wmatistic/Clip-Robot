package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.IntakeSampleChamberCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.PickupClipsCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.ScoreOnChamber;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.StowOuttakeSlides;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Sample;

@Autonomous
public class RedSpec extends OpMode {

    PinpointDrive drive;
    TelemetryPacket tele;
    SequentialAction path;
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

        path = createPath();

        targetedSample = new Sample(0,0,0,0);
        robot.limelightClass.targetSampleColor(1);

        robot.clipMech.setRightClipHolderTurns(0);

        firstRun = true;
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.autoPeriodic();

        if (firstRun) {

            if (robot.outtake.getTurns() != 0) {
                robot.outtake.setTurns(0);
            }

            firstRun = false;
        }

        telemetry.addData("Intake IK Turret Angle", Double.isNaN(IntakeInverseKinematics.turretAngle));
        telemetry.addData("Intake IK Slide Extension", IntakeInverseKinematics.slideExtension);
        telemetry.addData("Slide Extension Inches", IntakeInverseKinematics.slideExtensionInches);
        telemetry.addData("Left Clip Magazine Turns: ", robot.clipMech.getLeftClipHolderTurns());
        telemetry.addData("Right Clip Magazine Turns: ", robot.clipMech.getRightClipHolderTurns());
        telemetry.addData("Outtake Motor Current: ", robot.outtakeMotorOne.getCurrent(CurrentUnit.AMPS));

        path.run(tele);
    }

    public SequentialAction createPath() {
        return new SequentialAction(
            scorePreload(), loadClipsAndSpikeMark()
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
                        .lineToX(Poses.RedSpec.chamberBackUp)
                        .build()

        );
    }

    public SequentialAction loadClipsAndSpikeMark() {
        return new SequentialAction(
                drive.actionBuilder(drive.pinpoint.getPositionRR())
                        .splineToLinearHeading(Poses.RedSpec.pickupClipsFar, Math.toRadians(90))
                        .build(),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                    return false;
                },

                drive.actionBuilder(Poses.RedSpec.pickupClipsFar)
                        .lineToX(Poses.RedSpec.pickupClipsClose)
                        .build(),

                new SleepAction(1),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                    return false;
                }

//                telemetryPacket -> {
//                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                            new RefreshSampleCommand(),
//                            new WaitCommand(500),
//                            new InstantCommand(() -> targetedSample = robot.limelightClass.getTargetedSample()),
//                            new InstantCommand(() -> IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r)),
//                            new WaitCommand(100),
//                            new IntakeSampleCommand()
//                    ));
//                    return false;
//                }
        );
    }
}
