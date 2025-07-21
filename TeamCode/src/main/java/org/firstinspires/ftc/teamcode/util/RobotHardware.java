package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.ClipMech;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

@Config
public class RobotHardware {

    // Intake
    public AnalogInput turretServoInput;
    public CRServo turretServo;
    public PIDFController turretPID;
    public Servo intakeArmServo;
    public Servo intakeClawServo;
    public Servo clawRotationServo;
    public DcMotorEx intakeSlideMotor;
    public PIDFController intakeSlidePID;
    public ColorSensor intakeColorSensor;

    // Drivetrain
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;

    // Limelight
    public Limelight3A limelight;

    // Clip Mechanism
    public CRServo railServo;
    public AnalogInput railServoInput;
    public PIDFController railPID;
    public CRServo clipMagazineLeftServo;
    public AnalogInput clipMagazineLeftServoInput;
    public Servo clipMagazineLeftClawServo;
    public CRServo clipMagazineRightServo;
    public AnalogInput clipMagazineRightServoInput;
    public Servo clipMagazineRightClawServo;
    public PIDFController clipMagazinePID;
    public Servo clipPivotServo;

    // Outtake
    public DcMotorEx outtakeMotorOne, outtakeMotorTwo, outtakeMotorThree;
    public PIDFController outtakeSlideExtendPID, outtakeSlideRetractPID;
    public CRServo outtakeArmServo;
    public AnalogInput outtakeArmInput;
    public PIDFController outtakeArmPID;
    public Servo outtakeClawServo;
    public ColorSensor outtakeColorSensor;


    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    private boolean enabled;


    public Intake intake;
    public Drivetrain drivetrain;
    public ClipMech clipMech;
    public Outtake outtake;
    public Globals globals;


    public IntakeInverseKinematics intakeIK;
    public CameraCalculations cameraCalcs;
    public Limelight limelightClass;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        // ******************* INTAKE ******************* //
        this.turretServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.Intake.turretServoInput);
        this.turretServo = hardwareMap.crservo.get(RobotConstants.Intake.turretServo);
        this.turretPID = new PIDFController(RobotConstants.Intake.turretP, RobotConstants.Intake.turretI, RobotConstants.Intake.turretD, RobotConstants.Intake.turretF);

        this.intakeArmServo = hardwareMap.servo.get(RobotConstants.Intake.armServo);
        this.intakeArmServo.setPosition(RobotConstants.Intake.armStowed);

        this.intakeClawServo = hardwareMap.servo.get(RobotConstants.Intake.clawServo);
        this.intakeClawServo.setPosition(RobotConstants.Intake.clawOpen);

        this.clawRotationServo = hardwareMap.servo.get(RobotConstants.Intake.clawRotationServo);
        this.clawRotationServo.setPosition(RobotConstants.Intake.clawRotationStowed);

        this.intakeSlideMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeSlideMotor);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlidePID = new PIDFController(RobotConstants.Intake.slideP, RobotConstants.Intake.slideI, RobotConstants.Intake.slideD, RobotConstants.Intake.slideF);
        intakeSlideMotor.setPower(0);

        this.intakeColorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.Intake.intakeColorSensor);



        // ******************* DRIVETRAIN ******************* //
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftFront);
        leftRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftRear);
        rightRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightRear);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightFront);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();



        // ******************* LIMELIGHT ******************* //
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);



        // ******************* ClIP MECHANISM ******************* //
        this.railServo = hardwareMap.crservo.get(RobotConstants.ClipMech.railServo);
        this.railServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.ClipMech.railServoInput);
        this.railPID = new PIDFController(RobotConstants.ClipMech.railP, RobotConstants.ClipMech.railI, RobotConstants.ClipMech.railD, RobotConstants.ClipMech.railF);

        this.clipMagazineLeftServo = hardwareMap.crservo.get(RobotConstants.ClipMech.clipMagazineLeftServo);
        this.clipMagazineLeftServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.ClipMech.clipMagazineLeftServoInput);

        this.clipMagazineRightServo = hardwareMap.crservo.get(RobotConstants.ClipMech.clipMagazineRightServo);
        this.clipMagazineRightServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.ClipMech.clipMagazineRightServoInput);

        this.clipMagazinePID = new PIDFController(RobotConstants.ClipMech.clipMagazineP, RobotConstants.ClipMech.clipMagazineI, RobotConstants.ClipMech.clipMagazineD, RobotConstants.ClipMech.clipMagazineF);

        this.clipMagazineLeftClawServo = hardwareMap.servo.get(RobotConstants.ClipMech.clipMagazineLeftClawServo);
        this.clipMagazineLeftClawServo.setPosition(RobotConstants.ClipMech.clipMagazineClawClosed);

        this.clipMagazineRightClawServo = hardwareMap.servo.get(RobotConstants.ClipMech.clipMagazineRightClawServo);
        this.clipMagazineRightClawServo.setPosition(RobotConstants.ClipMech.clipMagazineClawClosed);

        this.clipPivotServo = hardwareMap.servo.get(RobotConstants.ClipMech.clipPivotServo);
        this.clipPivotServo.setPosition(RobotConstants.ClipMech.clipPivotTransfer);



        // ******************* OUTTAKE ******************* //
        this.outtakeMotorOne = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.outtakeMotorOne);
        this.outtakeMotorTwo = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.outtakeMotorTwo);
        this.outtakeMotorThree = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.outtakeMotorThree);

        outtakeMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotorOne.setPower(0);

        outtakeMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotorTwo.setPower(0);

        outtakeMotorThree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorThree.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotorThree.setPower(0);

        outtakeMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        //outtakeMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotorThree.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeSlideExtendPID = new PIDFController(RobotConstants.Outtake.outtakeExtendingP, RobotConstants.Outtake.outtakeExtendingI, RobotConstants.Outtake.outtakeExtendingD, RobotConstants.Outtake.outtakeExtendingF);
        outtakeSlideRetractPID = new PIDFController(RobotConstants.Outtake.outtakeRetractingP, RobotConstants.Outtake.outtakeRetractingI, RobotConstants.Outtake.outtakeRetractingD, RobotConstants.Outtake.outtakeRetractingF);

        outtakeArmServo = hardwareMap.crservo.get(RobotConstants.Outtake.outtakeArmServo);
        outtakeArmInput = hardwareMap.get(AnalogInput.class, RobotConstants.Outtake.outtakeArmInput);
        outtakeArmPID = new PIDFController(RobotConstants.Outtake.armP, RobotConstants.Outtake.armI, RobotConstants.Outtake.armD, RobotConstants.Outtake.armF);

        outtakeClawServo = hardwareMap.servo.get(RobotConstants.Outtake.outtakeClawServo);
        outtakeClawServo.setPosition(RobotConstants.Outtake.clawClosed);

        outtakeColorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.Outtake.outtakeColorSensor);



        intake = new Intake();
        drivetrain = new Drivetrain();
        clipMech = new ClipMech();
        outtake = new Outtake();

        intakeIK = new IntakeInverseKinematics();
        limelightClass = new Limelight();
        cameraCalcs = new CameraCalculations();
    }

    public void periodic() {
        intake.periodic();
        drivetrain.periodic();
        clipMech.periodic();
        outtake.periodic();
    }

    public void autoPeriodic() {
        intake.periodic();
        clipMech.periodic();
        outtake.periodic();
    }
}
