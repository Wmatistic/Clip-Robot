package org.firstinspires.ftc.teamcode.util;

public class IntakeInverseKinematics {
    public static double turretAngle = RobotConstants.Intake.turretStowed;
    public static int slideExtension = RobotConstants.Intake.slideStowed;
    public static double clawRotation = RobotConstants.Intake.clawRotationStowed;
    public static double slideExtensionInches = 0.0;
    public static double turretAngleDeg = 0.0;

    public static void calculateIK(double x, double y, double r) {
        x += RobotConstants.IntakeIK.intakeXOffset;
        turretAngleDeg = getIKTurretAngleDeg(x, y);
        turretAngle = getIKTurretAngle(x, y);
        slideExtension = getIKSlideExtension(x, y);
        slideExtensionInches = getIKSlideExtensionInches(x, y);
        clawRotation = getClawRotation(r);
    }

    public static double getClawRotation(double r) {
        double tempTurretAngleDeg = turretAngleDeg;
        if (tempTurretAngleDeg > 180) {
            tempTurretAngleDeg -= 180;
        }
        return (r - tempTurretAngleDeg) / 180;
    }

    public static double getIKTurretAngle(double x, double y) {
        double temp = Math.toDegrees(Math.acos(x / RobotConstants.Intake.armLength));
        temp = ((temp) + 90) / 360;
        temp *= RobotConstants.IntakeIK.turretOffset;
        return temp;
    }

    public static double getIKTurretAngleDeg(double x, double y) {
        double temp = Math.toDegrees(Math.acos(x / RobotConstants.Intake.armLength));
        return ((temp) + 90);
    }

    public static int getIKSlideExtension(double x, double y) {
        return inchesToMotorTicks(y - Math.sqrt(Math.pow(RobotConstants.Intake.armLength, 2) - Math.pow(x, 2)));
    }

    public static double getIKSlideExtensionInches(double x, double y) {
        return y - Math.sqrt(Math.pow(RobotConstants.Intake.armLength, 2) - Math.pow(x, 2));
    }

    public static int inchesToMotorTicks(double inches) {
        int result = (int) (inches / .024);

        result *= RobotConstants.IntakeIK.slideOffset;

        return result;
    }
}
