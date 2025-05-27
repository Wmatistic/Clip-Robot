package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Config
public class RobotHardware {

    // Intake
    private AnalogInput turretServoInput;
    private CRServo turretServo;


    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    private boolean enabled;


    public Intake intake;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.turretServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.Intake.turretServoInput);
        this.turretServo = hardwareMap.crservo.get(RobotConstants.Intake.turretServo);
    }
}
