package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.Params;

public class Drivetrain implements Subsystem {
    private RobotHardware robot;

    private GamepadEx driver;

    private double y, x, rx, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading, rotX, rotY;

    public Drivetrain() {
        this.robot = RobotHardware.getInstance();
    }

    public void setDriver(GamepadEx driver) {
        this.driver = driver;
    }

    public void periodic() {

//        robot.pinpointDrive.setDrivePowers(new PoseVelocity2d(
//                new Vector2d(
//                        robot.driver.getLeftY(),
//                        -robot.driver.getLeftX()
//                ),
//                -robot.driver.getRightX()
//        ));

        y = driver.getLeftY() * 1.2;
        x = driver.getLeftX() * 1.2;
        rx = driver.getRightX() * 1.1;

        if (robot.clipMech.getClipMechState() == ClipMech.ClipMechState.LOAD_MAGAZINE_ONE || robot.clipMech.getClipMechState() == ClipMech.ClipMechState.LOAD_MAGAZINE_TWO || robot.clipMech.getClipMechState() == ClipMech.ClipMechState.LOAD_MAGAZINE_THREE || robot.clipMech.getClipMechState() == ClipMech.ClipMechState.LOAD_MAGAZINE_FOUR || robot.clipMech.getClipMechState() == ClipMech.ClipMechState.LOAD_MAGAZINE_FIVE) {
            y *= 0.3;
            x *= 0.3;
            rx *= 0.3;
        }


//        heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
//        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
//
//        leftFrontPower = (rotY + rotX + rx);
//        leftRearPower = (rotY - rotX + rx);
//        rightFrontPower = (rotY - rotX - rx);
//        rightRearPower = (rotY + rotX - rx);

        leftFrontPower = (y + x + rx);
        leftRearPower = (y - x + rx);
        rightFrontPower = (y - x - rx);
        rightRearPower = (y + x - rx);

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);

    }
}
