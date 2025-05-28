package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Config
public class RobotHardware {

    // Intake
    public AnalogInput turretServoInput;
    public CRServo turretServo;
    public PIDFController turretPID;
    public Servo armServo;
    public Servo clawRotationServo;
    public DcMotorEx intakeSlideMotor;
    public PIDFController intakeSlidePID;


    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    private boolean enabled;


    public Intake intake;
    public IntakeInverseKinematics intakeIK;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        intake = new Intake();

        this.turretServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.Intake.turretServoInput);
        this.turretServo = hardwareMap.crservo.get(RobotConstants.Intake.turretServo);
        this.turretPID = new PIDFController(RobotConstants.Intake.turretP, RobotConstants.Intake.turretI, RobotConstants.Intake.turretD, RobotConstants.Intake.turretF);
        this.armServo = hardwareMap.servo.get(RobotConstants.Intake.armServo);
        this.clawRotationServo = hardwareMap.servo.get(RobotConstants.Intake.clawRotationServo);

        this.intakeSlideMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeSlideMotor);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeSlidePID = new PIDFController(RobotConstants.Intake.slideP, RobotConstants.Intake.slideI, RobotConstants.Intake.slideD, RobotConstants.Intake.slideF);
        intakeSlideMotor.setPower(0);
    }

    public void periodic() {
        intake.periodic();
    }
}
