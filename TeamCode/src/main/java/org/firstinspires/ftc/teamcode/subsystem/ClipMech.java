package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipMech implements Subsystem {
    private final RobotHardware robot;

    private double railTarget;

    public ClipMech() {
        this.robot = RobotHardware.getInstance();

        this.railTarget = 0.0;
    }

    @Override
    public void periodic() {
        updateRail();
    }

    private void updateRail() {
        double correction = robot.railPID.calculate(getRailPosition(), getRailTarget());
        robot.railServo.setPower(-correction);
    }

    public double getRailPosition() {
        return robot.railServoInput.getVoltage() / 3.3;
    }

    public double getRailTarget() {
        return railTarget;
    }

    public void setRailTarget(double target) {
        railTarget = target;
    }
}
