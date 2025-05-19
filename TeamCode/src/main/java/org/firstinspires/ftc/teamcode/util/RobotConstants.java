package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public class RobotConstants {

    @Config
    public static class Intake {
        public static double armLength = 9.46;

        public static String turretServo = "turretServo";
        public static String turretServoInput = "turretServoInput";
        public static double turretP = 0.0;
        public static double turretI = 0.0;
        public static double turretD = 0.0;
        public static double turretF = 0.0;
    }
}
