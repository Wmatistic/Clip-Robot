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
        public static double slideResetAmpThreshold = 5.5;
        public static int slideMax = 700; // 450
        public static int slideStowed = 0;

        public static double turretP = 2;
        public static double turretI = 0.7;
        public static double turretD = 0.0001;
        public static double turretF = 0.0;
        public static double turretStowed = 0.37;
        public static double turretTransfer = 0.115;
        public static double turretChamber = 0.45;

        public static double armStowed = 0.05;
        public static double armIntake = 0.9;
        public static double armTransfer = 0.5;
        public static double armChamber = 0.6;

        public static double clawOpen = 0.0;
        public static double clawClosed = 0.55;

        public static double clawRotationStowed = 0.0;
        public static double clawRotationTransfer = 0.55;
        public static double clawRotationChamber = 0.4;

        // Color Sensor Values
        public static int upperRed = 1100;
        public static int lowerRed = 200;

        public static int upperGreen = 0;
        public static int lowerGreen = 0;

        public static int upperBlue = 1100;
        public static int lowerBlue = 300;
    }

    @Config
    public static class IntakeIK {
        public static double slideOffset = 0.85;
        public static double turretOffset = 1.15;
        public static double clawRotationOffset = -0.17;
        public static double intakeXOffset = -5.199;
        public static int underTicks = 70;
        public static double lowerIncreaseBy = 1.5;
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
        public static double railClippingRight = 1.02;
        public static double railClippingLeft = 0.3;
        public static double railSecureClipLeft = 0.75;
        public static double railSecureClipRight = 0.55;
        public static double railOutTheWayRight = 1.35;
        public static double railOutTheWayLeft = 0.0;

        public static double railFirstClip = 0.6;
        public static double railSecondClip = 0.37;
        public static double railThirdClip = 0.15;
        public static double railFourthClip = -0.05;
        public static double railFifthClip = 0.65;
        public static double railSixthClip = 0.9;
        public static double railSeventhClip = 1.15;
        public static double railEighthClip = 1.3;

        public static double clipMagazineP = 1.5;
        public static double clipMagazineI = 0.07;
        public static double clipMagazineD = 0.0;
        public static double clipMagazineF = 0.0;

        public static double clipMagazineStowed = 0.1;
        public static double clipMagazineLeftTransfer = 0.62;
        public static double clipMagazineRightTransfer = 0.645;
        public static double clipMagazinePickupInitial = 0.3;
        public static double clipMagazinePickupTouching = 0.125;
        public static double clipMagazinePickupLifted = -0.1;

        public static double clipMagazineClawClosed = 0.0;
        public static double clipMagazineClawOpen = 0.7;
        public static double clipMagazineClawHalfOpen = 0.35;

        public static double clipPivotUp = 0.63;
        public static double clipPivotDown = 0.0;
        public static double clipPivotTransfer = 0.25;
        public static double clipPivotOutTheWay = 0.15;
        public static double clipPivotPullOut  = 0.2;
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

        public static double outtakeRetractingP = 0.0034;
        public static double outtakeRetractingI = 0.0001;
        public static double outtakeRetractingD = 0.0;
        public static double outtakeRetractingF = 0.0;

        public static int slideStowed = 0;
        public static int slideTransfer = 1000;
        public static int slideChamber = 900;
        public static int slideChamberScoring = 280;

        public static double slideResetAmpThreshold = 1;



        public static double armP = 2.0;
        public static double armI = 0.0;
        public static double armD = 0.0;
        public static double armF = 0.0;

        public static double armStowed = 0.51;
        public static double armTransfer = 0.48;
        public static double armClip = -0.5;
        public static double armClipInter = 0.5;
        public static double armChamberScoreReady = 0.49;
        public static double armChamberScoreInitial = -0.2;
        public static double armChamberScoreFinal = 0.2;
        public static double armPreload = 0.4;



        public static double clawClosed = 0.5;
        public static double clawOpen = 0.27;



        // Color Sensor Values
        public static int upperRed = 2000;
        public static int lowerRed = 300;

        public static int upperGreen = 0;
        public static int lowerGreen = 0;

        public static int upperBlue = 2000;
        public static int lowerBlue = 500;
    }
}
