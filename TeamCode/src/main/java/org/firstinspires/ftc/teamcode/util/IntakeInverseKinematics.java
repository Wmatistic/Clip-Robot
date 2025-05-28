package org.firstinspires.ftc.teamcode.util;

public class IntakeInverseKinematics {
    public static double turretAngle = RobotConstants.Intake.turretStowed;
    public static int slideExtension = RobotConstants.Intake.slideStowed;

    public static void calculateIK(double x, double y) {
        setIKTurretAngle(x, y);
        setIKSlideExtension(x, y);
    }

    public static void setIKTurretAngle(double x, double y) {
        turretAngle = Math.acos(x / RobotConstants.Intake.armLength);
    }

    public static void setIKSlideExtension(double x, double y) {
        slideExtension = inchesToMotorTicks(y - Math.sqrt(Math.pow(x, 2) + Math.pow(RobotConstants.Intake.armLength, 2)));
    }

    public static int inchesToMotorTicks(double inches) {
        return (int) (inches * 100);
    }
}
