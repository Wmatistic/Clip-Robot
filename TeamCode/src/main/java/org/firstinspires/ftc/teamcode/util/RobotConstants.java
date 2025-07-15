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
        public static String intakeColorSensor = "intakeColorSensor";

        public static double slideP = 0.005;
        public static double slideI = 0.003;
        public static double slideD = 0.0;
        public static double slideF = 0.0;
        public static int slideMax = 450;
        public static int slideStowed = 0;

        public static double turretP = 1.5;
        public static double turretI = 0.01;
        public static double turretD = 0.0;
        public static double turretF = 0.0;
        public static double turretStowed = 0.37;
        public static double turretTransfer = 0.110;

        public static double armStowed = 0.05;
        public static double armIntake = 0.9;
        public static double armTransfer = 0.5;

        public static double clawOpen = 0.5;
        public static double clawClosed = 0.0;

        public static double clawRotationStowed = 0.0;
        public static double clawRotationTransfer = 0.6;

        // Color Sensor Values
        public static int upperRed = 1100;
        public static int lowerRed = 200;

        public static int upperGreen = 0;
        public static int lowerGreen = 0;

        public static int upperBlue = 0;
        public static int lowerBlue = 0;
    }

    @Config
    public static class IntakeIK {
        public static double slideOffset = 0.8;
        public static double turretOffset = 0.12;
        public static double clawRotationOffset = -0.1;
        public static double intakeXOffset = -5.199;
    }

    @Config
    public static class Drivetrain {
        public static String leftFront = "leftFront";
        public static String leftRear = "leftRear";
        public static String rightFront = "rightFront";
        public static String rightRear = "rightRear";
    }

    @Config
    public static class Limelight {

    }

    @Config
    public static class ClipMech {
        public static String railServo = "railServo";
        public static String railServoInput = "railServoInput";
        public static String clipMagazineLeftServo = "clipMagazineLeftServo";
        public static String clipMagazineLeftServoInput = "clipMagazineLeftServoInput";
        public static String clipMagazineLeftClawServo = "clipMagazineLeftClawServo";
        public static String clipMagazineRightServo = "clipMagazineRightServo";
        public static String clipMagazineRightServoInput = "clipMagazineRightServoInput";
        public static String clipMagazineRightClawServo = "clipMagazineRightClawServo";
        public static String clipPivotServo = "clipPivotServo";

        public static double railP = 2.0;
        public static double railI = 0.01;
        public static double railD = 0.0;
        public static double railF = 0.0;

        // Rail Positions are relative to rail, left and right are based on rail not clip holder
        public static double railStowed = 0.05;
        public static double railClippingRight = 1.0;
        public static double railClippingLeft = 0.25;
        public static double railSecureClipLeft = 0.60;
        public static double railSecureClipRight = 0.55;
        public static double railOutTheWayRight = 1.3;
        public static double railOutTheWayLeft = 0.0;

        public static double railFirstClip = 0.6;
        public static double railSecondClip = 0.37;
        public static double railThirdClip = 0.15;
        public static double railFourthClip = -0.05;
        public static double railFifthClip = 0.65;
        public static double railSixthClip = 0.9;
        public static double railSeventhClip = 1.15;
        public static double railEighthClip = 1.3;

        public static double clipMagazineP = 2;
        public static double clipMagazineI = 0.04;
        public static double clipMagazineD = 0.0;
        public static double clipMagazineF = 0.0;

        public static double clipMagazineStowed = 0.1;
        public static double clipMagazineTransfer = 0.63;

        public static double clipMagazineClawClosed = 0.0;
        public static double clipMagazineClawOpen = 0.7;
        public static double clipMagazineClawHalfOpen = 0.35;

        public static double clipPivotUp = 0.6;
        public static double clipPivotDown = 0.0;
        public static double clipPivotTransfer = 0.25;
        public static double clipPivotOutTheWay = 0.15;
    }

    @Config
    public static class Outtake {
        public static String outtakeMotorOne = "outtakeMotorOne";
        public static String outtakeMotorTwo = "outtakeMotorTwo";
        public static String outtakeMotorThree = "outtakeMotorThree";
        public static String outtakeArmServo = "outtakeArmServo";
        public static String outtakeArmInput = "outtakeArmInput";
        public static String outtakeClawServo = "outtakeClawServo";
        public static String outtakeColorSensor = "outtakeColorSensor";

        public static double outtakeExtendingP = 0.006;
        public static double outtakeExtendingI = 0.0;
        public static double outtakeExtendingD = 0.0;
        public static double outtakeExtendingF = 0.0;

        public static double outtakeRetractingP = 0.0044;
        public static double outtakeRetractingI = 0.0;
        public static double outtakeRetractingD = 0.0;
        public static double outtakeRetractingF = 0.0;

        public static int slideStowed = 0;
        public static int slideTransfer = 350;
        public static int slideChamber = 350;
        public static int slideChamberScoring = 200;



        public static double armP = 2.0;
        public static double armI = 0.0;
        public static double armD = 0.0;
        public static double armF = 0.0;

        public static double armStowed = 0.53;
        public static double armClip = -0.5;
        public static double armClipInter = 0.3;
        public static double armChamberScoreReady = 0.4;
        public static double armChamberScoreInitial = -0.2;
        public static double armChamberScoreFinal = -0;
        public static double armTest = -1;



        public static double clawClosed = 0.48;
        public static double clawOpen = 0.25;



        // Color Sensor Values
        public static int upperRed = 1100;
        public static int lowerRed = 300;

        public static int upperGreen = 0;
        public static int lowerGreen = 0;

        public static int upperBlue = 0;
        public static int lowerBlue = 0;
    }
}
