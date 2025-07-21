package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ClipMech implements Subsystem {
    private final RobotHardware robot;

    private double railTarget;
    private double leftClipHolderTarget, rightClipHolderTarget;
    private double prevLeftClipHolderPosition, prevRightClipHolderPosition, leftClipHolderTurns, rightClipHolderTurns;

    private double prevRailPosition, railTurns;

    private int currentClip;

    private ClipMechState clipMechState;

    public enum ClipMechState {
        CLIPPING, STOWED, LOAD_MAGAZINE_ONE, LOAD_MAGAZINE_TWO, LOAD_MAGAZINE_THREE, LOAD_MAGAZINE_FOUR, LOAD_MAGAZINE_FIVE
    }

    public enum ClipHolder {
        LEFT, RIGHT, BOTH
    }

    public ClipMech() {
        this.robot = RobotHardware.getInstance();

        this.prevRailPosition = getRealRailPosition();
        this.railTurns = 0;

        this.railTarget = RobotConstants.ClipMech.railStowed;

        this.leftClipHolderTarget = RobotConstants.ClipMech.clipMagazineStowed;
        this.rightClipHolderTarget = RobotConstants.ClipMech.clipMagazineStowed;

        this.prevLeftClipHolderPosition = getRealLeftClipMagazinePosition();
        this.prevRightClipHolderPosition = getRealRightClipMagazinePosition();
        this.leftClipHolderTurns = 0;
        this.rightClipHolderTurns = 0;

        this.currentClip = 1;

        this.clipMechState = ClipMechState.STOWED;
    }

    @Override
    public void periodic() {
        updateRail();
        updateClipMagazines();
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

    public void setRightClipHolderTurns(double turns) {
        rightClipHolderTurns = turns;
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
            case 5:
                return RobotConstants.ClipMech.railFifthClip;
            case 6:
                return RobotConstants.ClipMech.railSixthClip;
            case 7:
                return RobotConstants.ClipMech.railSeventhClip;
            case 8:
                return RobotConstants.ClipMech.railEighthClip;
            default:
                return railTarget;
        }
    }

    public int getCurrentClip() {
        return currentClip;
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

    public double getClippingPosition() {
        if (getCurrentClipMagazine() == ClipHolder.LEFT) {
            return RobotConstants.ClipMech.railClippingRight;
        } else if (getCurrentClipMagazine() == ClipHolder.RIGHT) {
            return RobotConstants.ClipMech.railClippingLeft;
        } else {
            return RobotConstants.ClipMech.railClippingRight;
        }
    }

    public double getClipSecurePosition() {
        if (getCurrentClipMagazine() == ClipHolder.LEFT) {
            return RobotConstants.ClipMech.railSecureClipLeft;
        } else if (getCurrentClipMagazine() == ClipHolder.RIGHT) {
            return RobotConstants.ClipMech.railSecureClipRight;
        } else {
            return RobotConstants.ClipMech.railSecureClipLeft;
        }
    }

    public double getOutTheWayPosition() {
        if (getCurrentClipMagazine() == ClipHolder.LEFT) {
            return RobotConstants.ClipMech.railOutTheWayRight;
        } else if (getCurrentClipMagazine() == ClipHolder.RIGHT) {
            return RobotConstants.ClipMech.railOutTheWayLeft;
        } else {
            return RobotConstants.ClipMech.railOutTheWayRight;
        }
    }

    public ClipHolder getCurrentClipMagazine() {
        if (currentClip > 4) {
            return ClipHolder.RIGHT;
        } else if (currentClip <= 4 && currentClip > 0){
            return ClipHolder.LEFT;
        } else {
            return ClipHolder.LEFT;
        }
    }

    public void updateClipMagazines() {
        double leftCorrection = robot.clipMagazinePID.calculate(getLeftClipMagazinePosition(), getLeftClipMagazineTarget());
        robot.clipMagazineLeftServo.setPower(-leftCorrection);

        double rightCorrection = robot.clipMagazinePID.calculate(getRightClipMagazinePosition(), getRightClipMagazineTarget());
        robot.clipMagazineRightServo.setPower(-rightCorrection);
    }

    public void setClipHolderMagazines(ClipHolder clipHolder, double target) {
        switch (clipHolder) {
            case LEFT:
                setLeftClipMagazineTarget(target);
                break;
            case RIGHT:
                setRightClipMagazineTarget(target);
                break;
            case BOTH:
                setLeftClipMagazineTarget(target);
                setRightClipMagazineTarget(target);
                break;
        }
    }

    public void setClipHolderMagazineClaws(ClipHolder clipHolder, double position) {
        switch (clipHolder) {
            case LEFT:
                robot.clipMagazineLeftClawServo.setPosition(position);
                break;
            case RIGHT:
                robot.clipMagazineRightClawServo.setPosition(position);
                break;
            case BOTH:
                robot.clipMagazineLeftClawServo.setPosition(position);
                robot.clipMagazineRightClawServo.setPosition(position);
                break;
        }
    }

    public double getRightClipMagazinePosition() {
        double position = getRealRightClipMagazinePosition();
        if (prevRightClipHolderPosition - position > 0.5) {
            rightClipHolderTurns++;
        } else if (prevRightClipHolderPosition - position < -0.5) {
            rightClipHolderTurns--;
        }

        prevRightClipHolderPosition = position;

        return position + rightClipHolderTurns;
    }

    public double getRealRightClipMagazinePosition() {
        return 1 - (robot.clipMagazineRightServoInput.getVoltage() / 3.3);
    }

    public double getRightClipMagazineTarget() {
        return rightClipHolderTarget;
    }

    public void setRightClipMagazineTarget(double target) {
        rightClipHolderTarget = target;
    }

    public double getLeftClipMagazinePosition() {
        double position = getRealLeftClipMagazinePosition();
        if (prevLeftClipHolderPosition - position > 0.5) {
            leftClipHolderTurns++;
        } else if (prevLeftClipHolderPosition - position < -0.5) {
            leftClipHolderTurns--;
        }

        prevLeftClipHolderPosition = position;

        return position + leftClipHolderTurns;
    }

    public double getRealLeftClipMagazinePosition() {
        return robot.clipMagazineLeftServoInput.getVoltage() / 3.3;
    }

    public double getLeftClipMagazineTarget() {
        return leftClipHolderTarget;
    }

    public void setLeftClipMagazineTarget(double target) {
        leftClipHolderTarget = target;
    }

    public double getLeftClipHolderTurns() {
        return leftClipHolderTurns;
    }

    public double getRightClipHolderTurns() {
        return rightClipHolderTurns;
    }

    public double getTurns() {
        return railTurns;
    }
}
