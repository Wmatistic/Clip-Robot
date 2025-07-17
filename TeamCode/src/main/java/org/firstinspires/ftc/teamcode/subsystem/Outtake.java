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
    private double prevArmPosition, armTurns;

    public boolean slideReset = false;

    private ClawState clawState;

    public enum ClawState {
        CLOSED, OPEN
    }

    private OuttakeState outtakeState;

    public enum OuttakeState {
        SCORING_CHAMBER_INITIAL, SCORING_CHAMBER_FINAL, STOWED, TRANSFERRING
    }

    public Outtake() {
        this.robot = RobotHardware.getInstance();

        outtakeState = OuttakeState.STOWED;

        slideTarget = RobotConstants.Outtake.slideStowed;
        armTarget = RobotConstants.Outtake.armStowed;

        this.prevArmPosition = getRealArmPosition();
        this.armTurns = 0;
    }

    public void periodic() {
        powerSlides();
        updateArm();
    }

    public void setOuttakeState(OuttakeState outtakeState) {
        this.outtakeState = outtakeState;
    }

    public OuttakeState getOuttakeState() {
        return outtakeState;
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
        int red = robot.outtakeColorSensor.red();
        int green = robot.outtakeColorSensor.green();
        int blue = robot.outtakeColorSensor.blue();

        return  red < RobotConstants.Outtake.upperRed && red > RobotConstants.Outtake.lowerRed ||
                green < RobotConstants.Outtake.upperGreen && green > RobotConstants.Outtake.lowerGreen ||
                blue < RobotConstants.Outtake.upperBlue && blue > RobotConstants.Outtake.lowerBlue;
    }

    public void updateArm() {
        double correction = -robot.outtakeArmPID.calculate(getArmPosition(), armTarget);

        robot.outtakeArmServo.setPower(correction);
    }

    public double getArmPosition() {
        double position = getRealArmPosition();

        if (prevArmPosition - position > 0.5) {
            armTurns++;
        } else if (prevArmPosition - position < -0.5) {
            armTurns--;
        }

        prevArmPosition = position;

        return position + armTurns;
    }

    public double getRealArmPosition() {
        return robot.outtakeArmInput.getVoltage() / 3.3;
    }

    public double getTurns() {
        return armTurns;
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
            robot.outtakeMotorOne.setPower(-1);
            robot.outtakeMotorTwo.setPower(-1);
            robot.outtakeMotorThree.setPower(-1);
        } else if (slideTarget == RobotConstants.Outtake.slideStowed && (robot.outtakeMotorOne.getCurrentPosition() - slideTarget) <= 20) {
            setOuttakeMotorsPower(0);
        } else {
            setOuttakeMotorsPower(correction);
        }
    }

    public void resetSlides(boolean slideReset) {
        this.slideReset = slideReset;
    }

    public void setOuttakeMotorsPower(double power) {
        robot.outtakeMotorOne.setPower(power);
        robot.outtakeMotorTwo.setPower(power);
        robot.outtakeMotorThree.setPower(power);
    }
}
