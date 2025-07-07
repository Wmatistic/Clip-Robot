package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipMech implements Subsystem {
    private final RobotHardware robot;

    private double railTarget;
    private double clipHolderTarget;

    private double prevRailPosition, railTurns;

    private int currentClip;

    private ClipMechState clipMechState;

    public enum ClipMechState {
        CLIPPING, STOWED
    }

    public ClipMech() {
        this.robot = RobotHardware.getInstance();

        this.prevRailPosition = getRealRailPosition();
        this.railTurns = 0;

        this.railTarget = RobotConstants.ClipMech.railStowed;
        this.clipHolderTarget = RobotConstants.ClipMech.clipHolderStowed;

        this.currentClip = 1;

        this.clipMechState = ClipMechState.STOWED;
    }

    @Override
    public void periodic() {
        updateRail();
        updateClipHolder();
    }

    public void setClipMechState(ClipMechState clipMechState) {
        this.clipMechState = clipMechState;
    }

    public ClipMechState getClipMechState() {
        return clipMechState;
    }

    private void updateRail() {
        double correction = robot.railPID.calculate(getRailPosition(), getRailTarget());
        robot.railServo.setPower(-correction);
    }

    public double getCurrentClipPosition() {
        switch (currentClip) {
            case 1:
                return RobotConstants.ClipMech.railFirstClip;
            case 2:
                return RobotConstants.ClipMech.railSecondClip;
            case 3:
                return RobotConstants.ClipMech.railThirdClip;
            case 4:
                return RobotConstants.ClipMech.railFourthClip;
            default:
                return railTarget;
        }
    }

    public void incrementCurrentClip() {
        currentClip++;
    }

    public double getRealRailPosition() {
        return robot.railServoInput.getVoltage() / 3.3;
    }

    public double getRailPosition() {
        double position = getRealRailPosition();
        if (prevRailPosition - position > 0.5) {
            railTurns++;
        } else if (prevRailPosition - position < -0.5) {
            railTurns--;
        }

        prevRailPosition = position;

        return position + railTurns;
    }

    public double getRailTarget() {
        return railTarget;
    }

    public void setRailTarget(double target) {
        railTarget = target;
    }

    public void updateClipHolder() {
        double correction = robot.clipHolderPID.calculate(getClipHolderPosition(), getClipHolderTarget());
        robot.clipHolderServo.setPower(-correction);
    }

    public double getClipHolderPosition() {
        return robot.clipHolderServoInput.getVoltage() / 3.3;
    }

    public double getClipHolderTarget() {
        return clipHolderTarget;
    }

    public void setClipHolderTarget(double target) {
        clipHolderTarget = target;
    }

    public double getTurns() {
        return railTurns;
    }
}
