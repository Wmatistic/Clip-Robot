package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.service.controls.Control;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.clipmechcommand.ClipLoadedCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intakecommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.outtakecommand.OuttakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.ClipSampleCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.IntakeSampleChamberCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.LoadClipCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.PickupClipsCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.RegripSampleCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.ScoreOnChamber;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.StowOuttakeSlides;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Sample;

import java.util.Arrays;

// TODO:
//  1. fix isNaN error from being included in detected samples array
//  2. tune limelight values
//  3. fix structure of sample detection and intake related classes
//  4. change how we handle the slides extending to check for samples

@TeleOp
public class Cheesing extends CommandOpMode {

    public enum ControlMode {
        DEBUG, NORMAL
    }

    public ControlMode operatorControlMode, driverControlMode;

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver;
    private GamepadEx operator;
    private int x, y;
    private Sample targetedSample;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        operatorControlMode = ControlMode.NORMAL;
        driverControlMode = ControlMode.NORMAL;

        robot.init(hardwareMap);
        robot.drivetrain.setDriver(driver);

        robot.limelight.start();

        targetedSample = new Sample(0,0,0, 0);

        robot.outtake.updateSample();

        Globals.CLIP_MAGAZINES_LOADED = false;
        Globals.CLIP_LOADED = false;
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.periodic();
        driver.readButtons();
        operator.readButtons();



        telemetry.addData("Intake Slide Motor Power: ",robot.intakeSlideMotor.getPower());
        telemetry.addData("Intake Slide Motor Target", robot.intake.getExtensionTarget());
        //telemetry.addData("Turret Smth", robot.intake.getTurretPosition());
        telemetry.addData("Intake IK Turret Angle", IntakeInverseKinematics.turretAngle);
        telemetry.addData("Intake IK Turret Angle Deg: ", IntakeInverseKinematics.turretAngleDeg);
        telemetry.addData("Intake IK Slide Extension", IntakeInverseKinematics.slideExtension);
        telemetry.addData("Slide Extension Inches", IntakeInverseKinematics.slideExtensionInches);
        //telemetry.addData("Limelight Result", Arrays.toString(robot.limelightClass.getSampleLocations()));
        telemetry.addData("Sample Location X: ", targetedSample.x);
        telemetry.addData("Sample Location Y: ", targetedSample.y);
        telemetry.addData("Sample Rotation: ", targetedSample.r);
        telemetry.addData("Intake Slide Current: ", robot.intakeSlideMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake Slide Reset: ", Intake.slideReset);
        telemetry.addData("Intake Sample Slide Check: ", robot.intake.getSlideSampleCheck());
        telemetry.addData("Intake Slide Target: ", robot.intake.getExtensionTarget());
        telemetry.addData("Intake Slide Position: ", robot.intakeSlideMotor.getCurrentPosition());
//        telemetry.addData("Rail Servo Position: ", robot.clipMech.getRailPosition());
//        telemetry.addData("Rail Target Position: ", robot.clipMech.getRailTarget());
//        telemetry.addData("Rail Servo Power: ", robot.railServo.getPower());
//        telemetry.addData("Rail Turns: ", robot.clipMech.getTurns());
//        telemetry.addData("Outtake Arm Position: ", robot.outtake.getRealArmPosition());
//        telemetry.addData("Outtake Arm Power: ", robot.outtakeArmServo.getPower());
//        telemetry.addData("Outtake Arm Turns: ", robot.outtake.getTurns());
//        telemetry.addData("Left Clip Magazine Position: ", robot.clipMech.getRealLeftClipMagazinePosition());
//        telemetry.addData("Left Clip Magazine Power: ", robot.clipMagazineLeftServo.getPower());
//        telemetry.addData("Left Clip Magazine Turns: ", robot.clipMech.getLeftClipHolderTurns());
//        telemetry.addData("Right Clip Magazine Position: ", robot.clipMech.getRealRightClipMagazinePosition());
//        telemetry.addData("Right Clip Magazine Power: ", robot.clipMagazineRightServo.getPower());
//        telemetry.addData("Right Clip Magazine Turns: ", robot.clipMech.getRightClipHolderTurns());
        //telemetry.addData("Intake Color Sensor Red: ", robot.intakeColorSensor.red());
        //telemetry.addData("Intake Color Sensor Green: ", robot.intakeColorSensor.green());
        //telemetry.addData("Intake Color Sensor Blue: ", robot.intakeColorSensor.blue());
//        telemetry.addData("Outtake Color Sensor Red: ", robot.outtakeColorSensor.red());
//        telemetry.addData("Outtake Color Sensor Green: ", robot.outtakeColorSensor.green());
//        telemetry.addData("Outtake Color Sensor Blue: ", robot.outtakeColorSensor.blue());
        telemetry.addData("Current Clip: ", robot.clipMech.getCurrentClip());
        telemetry.addData("Clip Mech State: ", robot.clipMech.getClipMechState());
        telemetry.addData("Outtake Motor Position: ", robot.outtake.getSlideCurrentPosition());
        telemetry.addData("Outtake Motor One Power: ", robot.outtakeMotorOne.getPower());
        telemetry.addData("Outtake Motor Two Power: ", robot.outtakeMotorTwo.getPower());
        telemetry.addData("Outtake Motor Three Power: ", robot.outtakeMotorThree.getPower());
        telemetry.addData("Samples Visible: ", robot.limelightClass.getSamples().size());
        telemetry.update();

        // Load Clip and Clip Sample Recursive Checks
        if (!Globals.CLIP_LOADED && !Globals.SAMPLE_LOADED && robot.outtake.getOuttakeState() == Outtake.OuttakeState.STOWED && Globals.CLIP_MAGAZINES_LOADED && robot.clipMech.getClipMechState() == ClipMech.ClipMechState.STOWED) {
            CommandScheduler.getInstance().schedule(new LoadClipCommand());
        }

        if (Globals.CLIP_LOADED && Globals.SAMPLE_LOADED && robot.outtake.getOuttakeState() == Outtake.OuttakeState.STOWED && robot.clipMech.getClipMechState() == ClipMech.ClipMechState.STOWED) {
            CommandScheduler.getInstance().schedule(new ClipSampleCommand());
        }

        // Calculate Inverse Kinematics for Targeted Sample
        if (robot.limelightClass.hasSamples()) {
            targetedSample = robot.limelightClass.getTargetedSample();
            IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r);
        }


        // Operator Switch Control Modes
        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            if (operatorControlMode == ControlMode.NORMAL) {
                operatorControlMode = ControlMode.DEBUG;
            } else if (operatorControlMode == ControlMode.DEBUG) {
                operatorControlMode = ControlMode.NORMAL;
            }

            operator.gamepad.rumble(150);
        }

        // Operator Control Modes
        switch (operatorControlMode) {
            case NORMAL:



                break;

            case DEBUG:

                // Increment / Decrement Current Clip
                if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    robot.clipMech.decrementCurrentClip();
                }

                if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    robot.clipMech.incrementCurrentClip();
                }

                // Retry Load Clip
                if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new ClipLoadedCommand(false),
                                    new LoadClipCommand()
                            )
                    );
                }


                // Retry Clip
                if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new OuttakeStateCommand(Outtake.OuttakeState.STOWED),
                                    new ClipSampleCommand()
                            )
                    );
                }

                break;
        }

        // Drive Change Control Mode
        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            if (driverControlMode == ControlMode.NORMAL) {
                driverControlMode = ControlMode.DEBUG;
            } else if (driverControlMode == ControlMode.DEBUG) {
                driverControlMode = ControlMode.NORMAL;
            }

            driver.gamepad.rumble(150);
        }

        switch (driverControlMode) {
            case NORMAL:

                if (driver.wasJustPressed(GamepadKeys.Button.X)) {
                    Globals.CLIP_MAGAZINES_LOADED = true;
                }

        //
        //        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
        //            robot.intake.setExtensionTarget(IntakeInverseKinematics.slideExtension);
        //            robot.intake.setTurretTarget(IntakeInverseKinematics.turretAngle);
        //            robot.intakeArmServo.setPosition(RobotConstants.Intake.armIntake);
        //            robot.clawRotationServo.setPosition(IntakeInverseKinematics.clawRotation);
        //        }
        //        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
        //            robot.intake.setExtensionTarget(RobotConstants.Intake.slideStowed);
        //            robot.intake.setTurretTarget(RobotConstants.Intake.turretStowed);
        //            robot.intakeArmServo.setPosition(RobotConstants.Intake.armStowed);
        //            robot.clawRotationServo.setPosition(RobotConstants.Intake.clawRotationStowed);
        //        }

                if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && robot.limelightClass.hasSamples()) {
                    CommandScheduler.getInstance().schedule(new IntakeSampleCommand());
                }

                if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                    if (robot.intake.getIntakeState() == Intake.IntakeState.STOWED) {
                        robot.intake.setIntakeState(Intake.IntakeState.INTAKING_CHAMBER_1);
                    }

                    CommandScheduler.getInstance().schedule(new IntakeSampleChamberCommand());
                }

                if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2 && robot.intake.getIntakeState() == Intake.IntakeState.INTAKING_CHAMBER_2) {
                    robot.intake.setIntakeState(Intake.IntakeState.INTAKING_CHAMBER_3);

                    CommandScheduler.getInstance().schedule(new IntakeSampleChamberCommand());
                }

                if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && Globals.CLIP_LOADED && robot.outtake.getOuttakeState() == Outtake.OuttakeState.STOWED) {
                    CommandScheduler.getInstance().schedule(new TransferSampleCommand());
                }

        //        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
        //            robot.intake.setSlideSampleCheck(-50);
        //            robot.intake.setExtensionTarget(IntakeInverseKinematics.slideExtension);
        //        }

                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    robot.limelightClass.targetNextSample();
                }

                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    robot.limelightClass.targetSampleColor(0);
                    robot.limelightClass.refreshSamples();
                }

                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    robot.limelightClass.targetSampleColor(1);
                    robot.limelightClass.refreshSamples();
                }

                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    robot.limelightClass.targetSampleColor(2);
                    robot.limelightClass.refreshSamples();
                }

                if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
                    robot.intake.resetSlides();
                }

                switch (robot.outtake.getOuttakeState()) {
                    case SCORING_CHAMBER_INITIAL:
                        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
                            CommandScheduler.getInstance().schedule(new ScoreOnChamber());
                        }

                        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
                            CommandScheduler.getInstance().schedule(new StowOuttakeSlides());
                        }

                        break;

                    case SCORING_CHAMBER_FINAL:
                        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
                            CommandScheduler.getInstance().schedule(new StowOuttakeSlides());
                        }

                        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
                            CommandScheduler.getInstance().schedule(new ScoreOnChamber());
                        }

                        break;

                    case STOWED:
                        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
                            CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                        }

                        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
                            switch (robot.clipMech.getClipMechState()) {
                                case LOAD_MAGAZINE_ONE:
                                    robot.clipMech.setClipMechState(ClipMech.ClipMechState.LOAD_MAGAZINE_THREE);
                                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                                case LOAD_MAGAZINE_TWO:
                                    robot.clipMech.setClipMechState(ClipMech.ClipMechState.STOWED);
                                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                                    break;
                                case LOAD_MAGAZINE_THREE:
                                    robot.clipMech.setClipMechState(ClipMech.ClipMechState.LOAD_MAGAZINE_TWO);
                                    CommandScheduler.getInstance().schedule(new PickupClipsCommand());
                                    break;
                                default:
                                    break;
                            }
                        }
                        break;
                }

                break;

            case DEBUG:

                if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new OuttakeStateCommand(Outtake.OuttakeState.STOWED),
                                    new RegripSampleCommand(),
                                    new ClipSampleCommand()
                            )
                    );
                }

                // Increment / Decrement Current Clip
                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    robot.clipMech.decrementCurrentClip();
                }

                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    robot.clipMech.incrementCurrentClip();
                }

                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    Globals.CLIP_LOADED = true;
                }

                break;
                }
    }

}
