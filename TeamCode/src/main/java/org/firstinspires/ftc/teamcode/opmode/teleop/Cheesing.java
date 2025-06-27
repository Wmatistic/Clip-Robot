package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Sample;

// TODO:
//  1. fix isNaN error from being included in detected samples array
//  2. tune limelight values
//  3. fix structure of sample detection and intake related classes
//  4. change how we handle the slides extending to check for samples

@TeleOp
public class Cheesing extends CommandOpMode {

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

        robot.init(hardwareMap, driver);

        robot.limelight.start();

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new TransferSampleCommand());

        targetedSample = new Sample(0,0,0);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.periodic();
        driver.readButtons();
        operator.readButtons();

        robot.limelightClass.refreshSamples();

        if (robot.limelightClass.hasSamples()) {
            targetedSample = robot.limelightClass.getTargetedSample();
            IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r);
        }

        telemetry.addData("Intake Slide Motor Power: ",robot.intakeSlideMotor.getPower());
        telemetry.addData("Intake Slide Motor Target", robot.intake.getExtensionTarget());
        telemetry.addData("Turret Smth", robot.intake.getTurretPosition());
        telemetry.addData("Intake IK Turret Angle", Double.isNaN(IntakeInverseKinematics.turretAngle));
        telemetry.addData("Intake IK Slide Extension", IntakeInverseKinematics.slideExtension);
        telemetry.addData("Slide Extension Inches", IntakeInverseKinematics.slideExtensionInches);
        //telemetry.addData("Limelight Result", Arrays.toString(robot.limelightClass.getSampleLocations()));
        telemetry.addData("Sample Location X: ", targetedSample.x);
        telemetry.addData("Sample Location Y: ", targetedSample.y);
        telemetry.addData("Sample Rotation: ", targetedSample.r);
        telemetry.addData("Intake Slide Current: ", robot.intakeSlideMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake Slide Reset: ", Intake.slideReset);
        telemetry.addData("Intake Sample Slide Check: ", Intake.slideSampleCheck);
        telemetry.addData("Rail Servo Position: ", robot.clipMech.getRailPosition());
        telemetry.addData("Rail Target Position: ", robot.clipMech.getRailTarget());
        telemetry.addData("Rail Servo Power: ", robot.railServo.getPower());
        telemetry.addData("Outtake Motor Power: ", robot.outtakeMotorOne.getPower());
        telemetry.addData("Outtake Motor Position: ", robot.outtakeMotorOne.getCurrentPosition());
        telemetry.update();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.intake.setExtensionTarget(IntakeInverseKinematics.slideExtension);
            robot.intake.setTurretTarget(IntakeInverseKinematics.turretAngle);
            robot.intakeArmServo.setPosition(RobotConstants.Intake.armIntake);
            robot.clawRotationServo.setPosition(IntakeInverseKinematics.clawRotation);
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            robot.intake.setExtensionTarget(RobotConstants.Intake.slideStowed);
            robot.intake.setTurretTarget(RobotConstants.Intake.turretStowed);
            robot.intakeArmServo.setPosition(RobotConstants.Intake.armStowed);
            robot.clawRotationServo.setPosition(RobotConstants.Intake.clawRotationStowed);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            CommandScheduler.getInstance().schedule(new IntakeSampleCommand());
        }

        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            Intake.slideSampleCheck = 200;
            robot.intake.setExtensionTarget(0);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            robot.limelightClass.targetNextSample();
        }

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            robot.intake.resetSlides();
        }

//        if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
//            robot.clipMech.setRailTarget(1.0);
//        }
//        if (operator.wasJustPressed(GamepadKeys.Button.B)) {
//            robot.clipMech.setRailTarget(0.0);
//        }
//
//        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
//            robot.clipHolderServo.setPosition(1);
//        }
//        if (operator.wasJustPressed(GamepadKeys.Button.X)) {
//            robot.clipHolderServo.setPosition(0);
//        }


        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            robot.outtake.setSlideTarget(RobotConstants.Outtake.slideTransfer);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            robot.outtake.setSlideTarget(RobotConstants.Outtake.slideStowed);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            robot.outtake.setArmTarget(RobotConstants.Outtake.armTest);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            robot.outtake.setArmTarget(RobotConstants.Outtake.armStowed);
        }
    }

}
