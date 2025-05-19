package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class Limelight {
    Limelight3A limelight;

    LLResult result;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public ArrayList<Double> getSampleLocations() {
        result = limelight.getLatestResult();
        double[] samples = result.getPythonOutput();


        return new ArrayList<>();
    }
}