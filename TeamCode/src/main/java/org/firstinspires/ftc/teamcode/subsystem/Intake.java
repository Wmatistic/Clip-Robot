package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class Intake implements Subsystem {

    private AnalogInput turretServoInput;
    private CRServo turretServo;
    private PIDFController turretPID;
    private double turretTarget;
    private ClawState clawState;

    public enum IntakeState {
        INTAKING, TRANSFERRING, STOWED
    }

    public enum ClawState {
        CLOSED, OPEN
    }

    public Intake(HardwareMap hardwareMap) {
        turretPID = new PIDFController(RobotConstants.Intake.turretP, RobotConstants.Intake.turretI, RobotConstants.Intake.turretD, RobotConstants.Intake.turretF);
    }

    public void setClawState(ClawState state) {
        clawState = state;
    }

    public void setState(IntakeState state) {

    }

    public double getIKTurretAngle(double x, double y) {
        return Math.acos(x / RobotConstants.Intake.armLength);
    }

    public int getIKSlideExtension(double x, double y) {
        return inchesToMotorTicks(y - Math.sqrt(Math.pow(x, 2) + Math.pow(RobotConstants.Intake.armLength, 2)));
    }

    public void updateAssembly() {
        updateTurret();
    }

    private void updateTurret() {
        double correction = -turretPID.calculate(getTurretPosition(), turretTarget);

        turretServo.setPower(correction);
    }

    public double getTurretPosition() {
        return turretServoInput.getVoltage() / 3.3;
    }

    public void setTurretTarget(double target) {
        turretTarget = target;
    }

    private int inchesToMotorTicks(double inches) {
        return (int) (inches * 100);
    }
}
