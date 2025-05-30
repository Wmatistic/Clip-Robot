package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Limelight;

@Config
public class RobotHardware {

    // Intake
    public AnalogInput turretServoInput;
    public CRServo turretServo;
    public PIDFController turretPID;
    public Servo armServo;
    public Servo clawServo;
    public Servo clawRotationServo;
    public DcMotorEx intakeSlideMotor;
    public PIDFController intakeSlidePID;

    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;

    public Limelight3A limelight;

    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    private boolean enabled;

    public GamepadEx driver;

    public Intake intake;
    public Drivetrain drivetrain;
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

    public void init(final HardwareMap hardwareMap, GamepadEx driver) {
        this.hardwareMap = hardwareMap;

        this.driver = driver;

        // ******************* INTAKE ******************* //
        this.turretServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.Intake.turretServoInput);
        this.turretServo = hardwareMap.crservo.get(RobotConstants.Intake.turretServo);
        this.turretPID = new PIDFController(RobotConstants.Intake.turretP, RobotConstants.Intake.turretI, RobotConstants.Intake.turretD, RobotConstants.Intake.turretF);
        this.armServo = hardwareMap.servo.get(RobotConstants.Intake.armServo);
        this.armServo.setPosition(RobotConstants.Intake.armStowed);

        this.clawServo = hardwareMap.servo.get(RobotConstants.Intake.clawServo);
        this.clawServo.setPosition(RobotConstants.Intake.clawOpen);
//        this.clawRotationServo = hardwareMap.servo.get(RobotConstants.Intake.clawRotationServo);

        this.intakeSlideMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeSlideMotor);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeSlidePID = new PIDFController(RobotConstants.Intake.slideP, RobotConstants.Intake.slideI, RobotConstants.Intake.slideD, RobotConstants.Intake.slideF);
        intakeSlideMotor.setPower(0);

        // ******************* DRIVETRAIN ******************* //
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftFront);
        leftRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftRear);
        rightRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightRear);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightFront);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        // ******************* LIMELIGHT ******************* //
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        intake = new Intake();
        intakeIK = new IntakeInverseKinematics();
        drivetrain = new Drivetrain();
        limelightClass = new Limelight();
        cameraCalcs = new CameraCalculations();
    }

    public void periodic() {
        intake.periodic();
        drivetrain.periodic();
    }
}
