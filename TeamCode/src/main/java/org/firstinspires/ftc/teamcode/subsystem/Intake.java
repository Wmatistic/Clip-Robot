package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Intake implements Subsystem {
    private final RobotHardware robot;
    private double turretTarget;
    private ClawState clawState;
    private IntakeState intakeState;
    private int target, prevTarget;

    public static boolean slideReset = false;

    private int slideSampleCheck;

    public enum IntakeState {
        INTAKING, TRANSFERRING, STOWED, INTAKING_CHAMBER_1, INTAKING_CHAMBER_2, INTAKING_CHAMBER_3
    }

    public enum ClawState {
        CLOSED, OPEN
    }

    public Intake() {
        this.robot = RobotHardware.getInstance();

        setExtensionTarget(RobotConstants.Intake.slideStowed);
        setTurretTarget(RobotConstants.Intake.turretStowed);
        slideSampleCheck = 0;

        clawState = ClawState.OPEN;
        intakeState = IntakeState.STOWED;
    }

    public void setClawState(ClawState state) {
        clawState = state;
        switch (state) {
            case OPEN:
                robot.intakeClawServo.setPosition(RobotConstants.Intake.clawOpen);
                break;
            case CLOSED:
                robot.intakeClawServo.setPosition(RobotConstants.Intake.clawClosed);
                break;
        }
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void setIntakeState(IntakeState state) {
        this.intakeState = state;
    }

    public void periodic() {
        updateTurret();
        powerSlides();
    }

    public boolean isSample() {
        int red = robot.intakeColorSensor.red();
        int green = robot.intakeColorSensor.green();
        int blue = robot.intakeColorSensor.blue();

        return  red < RobotConstants.Intake.upperRed && red > RobotConstants.Intake.lowerRed ||
                green < RobotConstants.Intake.upperGreen && green > RobotConstants.Intake.lowerGreen ||
                blue < RobotConstants.Intake.upperBlue && blue > RobotConstants.Intake.lowerBlue;
    }

    public int getSlideSampleCheck() {
        return slideSampleCheck;
    }

    public void setSlideSampleCheck(int slideSampleCheck) {
        this.slideSampleCheck = slideSampleCheck;
    }

    public void setExtensionTarget(int position) {
        position += slideSampleCheck;
        if (position <= RobotConstants.Intake.slideMax && position >= RobotConstants.Intake.slideStowed) {
            prevTarget = target;
            target = position;
        }
    }

    public int getExtensionTarget() {
        return target;
    }

    public void powerSlides() {
        double correction;

        correction = robot.intakeSlidePID.calculate(robot.intakeSlideMotor.getCurrentPosition(), target);

        if (slideReset) {
            robot.intakeSlideMotor.setPower(-1);
            if (robot.intakeSlideMotor.getCurrent(CurrentUnit.AMPS) > RobotConstants.Intake.slideResetAmpThreshold) {
                robot.intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.intakeSlideMotor.setPower(0);

                slideReset = false;
            }
        } else if (target == RobotConstants.Intake.slideStowed && (robot.intakeSlideMotor.getCurrentPosition() - target) <= 20) {
            robot.intakeSlideMotor.setPower(0);
        } else {
            robot.intakeSlideMotor.setPower(correction);
        }
    }

    public void resetSlides() {
        slideReset = true;
    }

    private void updateTurret() {
        double correction = -robot.turretPID.calculate(getTurretPosition(), turretTarget);

        robot.turretServo.setPower(correction);
    }

    public double getTurretPosition() {
        return robot.turretServoInput.getVoltage() / 3.3;
    }

    public void setTurretTarget(double target) {
        turretTarget = target;
    }
}
