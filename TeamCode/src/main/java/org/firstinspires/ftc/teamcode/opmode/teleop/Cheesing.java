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
import org.firstinspires.ftc.teamcode.util.CameraCalculations;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

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

        robot.cameraCalcs.SampleToRealWorld(robot.limelightClass.getSampleLocations()[2], robot.limelightClass.getSampleLocations()[3]);

        IntakeInverseKinematics.calculateIK(CameraCalculations.worldPositionX-5.199, CameraCalculations.worldPositionY);

        telemetry.addData("Intake Slide Motor Power: ",robot.intakeSlideMotor.getPower());
        telemetry.addData("Intake Slide Motor Target", robot.intake.getExtensionTarget());
        telemetry.addData("Turret Smth", robot.intake.getTurretPosition());
        telemetry.addData("Intake IK Turret Angle", IntakeInverseKinematics.turretAngle);
        telemetry.addData("Intake IK Slide Extension", IntakeInverseKinematics.slideExtension);
        telemetry.addData("Slide Extension Inches", IntakeInverseKinematics.slideExtensionInches);
        //telemetry.addData("Limelight Result", Arrays.toString(robot.limelightClass.getSampleLocations()));
        telemetry.addData("Sample Location X: ", CameraCalculations.worldPositionX);
        telemetry.addData("Sample Location Y: ", CameraCalculations.worldPositionY);
        telemetry.update();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.intake.setExtensionTarget(IntakeInverseKinematics.slideExtension);
            robot.intake.setTurretTarget(IntakeInverseKinematics.turretAngle);
            robot.armServo.setPosition(RobotConstants.Intake.armIntake);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            CommandScheduler.getInstance().schedule(new IntakeSampleCommand());
        }
    }

}
