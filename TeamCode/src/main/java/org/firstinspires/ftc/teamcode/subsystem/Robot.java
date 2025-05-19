package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    public Intake intake;

    public Robot(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
    }

    public void updateAssembly() {
        intake.updateAssembly();
    }
}
