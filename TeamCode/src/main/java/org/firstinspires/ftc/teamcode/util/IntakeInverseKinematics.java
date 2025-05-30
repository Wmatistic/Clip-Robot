package org.firstinspires.ftc.teamcode.util;

public class IntakeInverseKinematics {
    public static double turretAngle = RobotConstants.Intake.turretStowed;
    public static int slideExtension = RobotConstants.Intake.slideStowed;
    public static double clawRotation = RobotConstants.Intake.clawRotationStowed;
    public static double slideExtensionInches = 0.0;
    public static int slideOffset = 0;
    public static double turretAngleDeg = 0.0;
    public static double turretOffset = 0.1;
    public static double clawRotationOffset = 90;

    public static void calculateIK(double x, double y, double r) {
        setIKTurretAngle(x, y);
        setIKSlideExtension(x, y);
        setClawRotation(r);
    }

    public static void setClawRotation(double r) {
        clawRotation = ((r + clawRotationOffset) + turretAngleDeg) / 360;
    }

    public static void setIKTurretAngle(double x, double y) {
        turretAngle = Math.toDegrees(Math.acos(x / RobotConstants.Intake.armLength));
        turretAngleDeg = ((turretAngle) + 90);
        turretAngle = ((turretAngle) + 90) / 360;
        turretAngle += turretOffset;
    }

    public static void setIKSlideExtension(double x, double y) {
        slideExtension = inchesToMotorTicks(y - Math.sqrt(Math.pow(RobotConstants.Intake.armLength, 2) - Math.pow(x, 2)));
    }

    public static int inchesToMotorTicks(double inches) {
        slideExtensionInches = inches;
        return (int) (inches / .024) - slideOffset;
    }
}
