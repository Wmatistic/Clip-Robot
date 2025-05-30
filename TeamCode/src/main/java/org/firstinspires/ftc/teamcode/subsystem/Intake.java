package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Intake implements Subsystem {
    private final RobotHardware robot;
    private double turretTarget;
    private ClawState clawState;
    private int target, prevTarget;

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
    }

    public void setClawState(ClawState state) {
        clawState = state;
        switch (state) {
            case OPEN:
                robot.clawServo.setPosition(RobotConstants.Intake.clawOpen);
                break;
            case CLOSED:
                robot.clawServo.setPosition(RobotConstants.Intake.clawClosed);
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
        prevTarget = target;
        target = position;
    }

    public int getExtensionTarget() {
        return target;
    }

    public void powerSlides() {
        double correction;

        correction = robot.intakeSlidePID.calculate(robot.intakeSlideMotor.getCurrentPosition(), target);

        robot.intakeSlideMotor.setPower(correction);
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
