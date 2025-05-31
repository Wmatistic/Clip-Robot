package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.teleopcommand.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.util.CameraCalculations;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Sample;

import java.util.Arrays;

@TeleOp
public class Cheesing extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver;
    private int x, y;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driver = new GamepadEx(gamepad1);

        robot.init(hardwareMap, driver);

        robot.limelight.start();

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new TransferSampleCommand());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.periodic();
        driver.readButtons();

        robot.limelightClass.refreshSamples();

        if (robot.limelightClass.hasSamples()) {
            Sample targetedSample = robot.limelightClass.getTargetedSample();
            IntakeInverseKinematics.calculateIK(targetedSample.x, targetedSample.y, targetedSample.r);
        }

        telemetry.addData("Intake Slide Motor Power: ",robot.intakeSlideMotor.getPower());
        telemetry.addData("Intake Slide Motor Target", robot.intake.getExtensionTarget());
        telemetry.addData("Turret Smth", robot.intake.getTurretPosition());
        telemetry.addData("Intake IK Turret Angle", IntakeInverseKinematics.turretAngle);
        telemetry.addData("Intake IK Slide Extension", IntakeInverseKinematics.slideExtension);
        telemetry.addData("Slide Extension Inches", IntakeInverseKinematics.slideExtensionInches);
        telemetry.addData("Limelight Result", Arrays.toString(robot.limelightClass.getSampleLocations()));
        telemetry.addData("Sample Location X: ", CameraCalculations.worldPositionX);
        telemetry.addData("Sample Location Y: ", CameraCalculations.worldPositionY);
        telemetry.update();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.intake.setExtensionTarget(IntakeInverseKinematics.slideExtension);
            robot.intake.setTurretTarget(IntakeInverseKinematics.turretAngle);
            robot.armServo.setPosition(RobotConstants.Intake.armIntake);
            robot.clawRotationServo.setPosition(IntakeInverseKinematics.clawRotation);
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            robot.intake.setExtensionTarget(RobotConstants.Intake.slideStowed);
            robot.intake.setTurretTarget(RobotConstants.Intake.turretStowed);
            robot.armServo.setPosition(RobotConstants.Intake.armStowed);
            robot.clawRotationServo.setPosition(RobotConstants.Intake.clawRotationStowed);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            CommandScheduler.getInstance().schedule(new IntakeSampleCommand());
        }

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            robot.limelightClass.targetNextSample();
        }
    }

}
