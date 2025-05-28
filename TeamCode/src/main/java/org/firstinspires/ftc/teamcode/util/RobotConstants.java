package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public class RobotConstants {

    @Config
    public static class Intake {
        public static double armLength = 9.46;

        public static String turretServo = "turretServo";
        public static String turretServoInput = "turretServoInput";
        public static String armServo = "armServo";
        public static String clawRotationServo = "clawRotationServo";
        public static String intakeSlideMotor;

        public static double slideP = 0.0;
        public static double slideI = 0.0;
        public static double slideD = 0.0;
        public static double slideF = 0.0;

        public static double turretP = 0.0;
        public static double turretI = 0.0;
        public static double turretD = 0.0;
        public static double turretF = 0.0;

        public static double turretStowed = 0.0;
        public static int slideStowed = 0;

        public static int armIntake = 0.5;
    }
}
