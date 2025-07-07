package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Outtake implements Subsystem {
    private final RobotHardware robot;

    private int slideTarget;
    private double armTarget;

    public static boolean slideReset = false;

    private ClawState clawState;

    public enum ClawState {
        CLOSED, OPEN
    }

    public Outtake() {
        this.robot = RobotHardware.getInstance();

        slideTarget = RobotConstants.Outtake.slideStowed;
        armTarget = RobotConstants.Outtake.armStowed;
    }

    public void periodic() {
        powerSlides();
        updateArm();
    }

    public void setClawState(ClawState state) {
        clawState = state;
        switch (state) {
            case OPEN:
                robot.outtakeClawServo.setPosition(RobotConstants.Outtake.clawOpen);
                break;
            case CLOSED:
                robot.outtakeClawServo.setPosition(RobotConstants.Outtake.clawClosed);
                break;
        }
    }

    public void updateSample() {
        Globals.SAMPLE_LOADED = isSample();
    }

    public boolean isSample() {
        NormalizedRGBA colors = robot.outtakeColorSensor.getNormalizedColors();

        return  colors.red < RobotConstants.Outtake.upperRed && colors.red > RobotConstants.Outtake.lowerRed &&
                colors.green < RobotConstants.Outtake.upperGreen && colors.green > RobotConstants.Outtake.lowerGreen &&
                colors.blue < RobotConstants.Outtake.upperBlue && colors.blue > RobotConstants.Outtake.lowerBlue;
    }

    public void updateArm() {
        double correction = -robot.outtakeArmPID.calculate(getArmPosition(), armTarget);

        robot.outtakeArmServo.setPower(correction);
    }

    public double getArmPosition() {
        return robot.outtakeArmInput.getVoltage() / 3.3;
    }

    public void setArmTarget(double armTarget) {
        this.armTarget = armTarget;
    }

    public void setSlideTarget(int slideTarget) {
        this.slideTarget = slideTarget;
    }

    public void powerSlides() {
        double correction;

        if (slideTarget > robot.outtakeMotorOne.getCurrentPosition()) {
            correction = robot.outtakeSlideExtendPID.calculate(robot.outtakeMotorOne.getCurrentPosition(), slideTarget);
        } else {
            correction = robot.outtakeSlideRetractPID.calculate(robot.outtakeMotorOne.getCurrentPosition(), slideTarget);
        }

        if (slideReset) {
//            robot.intakeSlideMotor.setPower(-1);
//            if (robot.intakeSlideMotor.getCurrent(CurrentUnit.AMPS) > 5) {
//                robot.intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.intakeSlideMotor.setPower(0);
//
//                slideReset = false;
//            }
        } else if (slideTarget == RobotConstants.Outtake.slideStowed && (robot.outtakeMotorOne.getCurrentPosition() - slideTarget) <= 20) {
            setOuttakeMotorsPower(0);
        } else {
            setOuttakeMotorsPower(correction);
        }
    }

    public void setOuttakeMotorsPower(double power) {
        robot.outtakeMotorOne.setPower(power);
        robot.outtakeMotorTwo.setPower(power);
        robot.outtakeMotorThree.setPower(power);
    }
}
