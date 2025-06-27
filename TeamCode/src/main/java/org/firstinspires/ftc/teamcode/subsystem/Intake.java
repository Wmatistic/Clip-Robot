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
    private int target, prevTarget;

    public static boolean slideReset = false;

    public static int slideSampleCheck;

    public enum IntakeState {
        INTAKING, TRANSFERRING, STOWED
    }

    public enum ClawState {
        CLOSED, OPEN
    }

    public Intake() {
        this.robot = RobotHardware.getInstance();

        setExtensionTarget(RobotConstants.Intake.slideStowed);
        setTurretTarget(RobotConstants.Intake.turretStowed);
        slideSampleCheck = 0;
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

    public void setState(IntakeState state) {

    }

    public void periodic() {
        updateTurret();
        powerSlides();
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
            if (robot.intakeSlideMotor.getCurrent(CurrentUnit.AMPS) > 5) {
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
