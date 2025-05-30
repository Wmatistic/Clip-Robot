package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public class RobotConstants {

    @Config
    public static class Intake {
        public static double armLength = 9.46;

        public static String turretServo = "turretServo";
        public static String turretServoInput = "turretServoInput";
        public static String armServo = "armServo";
        public static String clawServo = "clawServo";
        public static String clawRotationServo = "clawRotationServo";
        public static String intakeSlideMotor = "intakeSlideMotor";

        public static double slideP = .0055;
        public static double slideI = 0.0;
        public static double slideD = 0.0;
        public static double slideF = 0.0;
        public static int slideMax = 400;
        public static int slideStowed = 0;

        public static double turretP = 1.5;
        public static double turretI = 0.00001;
        public static double turretD = 0.0;
        public static double turretF = 0.0;
        public static double turretStowed = 0.59;
        public static double turretTransfer = 0.085;

        public static double armStowed = 0.0;
        public static double armIntake = 0.9;
        public static double armTransfer = 0.6;

        public static double clawOpen = 0.5;
        public static double clawClosed = 0.0;

        public static double clawRotationStowed = 0.0;
    }

    @Config
    public static class Drivetrain {
        public static String leftFront = "leftFront";
        public static String leftRear = "leftRear";
        public static String rightFront = "rightFront";
        public static String rightRear = "rightRear";
    }
}
