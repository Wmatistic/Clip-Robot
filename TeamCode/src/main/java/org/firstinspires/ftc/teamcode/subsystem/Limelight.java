package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.ArrayList;

public class Limelight {

    RobotHardware robot;
    public static LLResult result;

    public Limelight() {
        this.robot = RobotHardware.getInstance();
        result = robot.limelight.getLatestResult();
    }

    public void refreshResult() {
        result = robot.limelight.getLatestResult();
    }

    public double[] getSampleLocations() {
        result = robot.limelight.getLatestResult();
        return result.getPythonOutput();
    }

    public double getSampleRotation() {
        return result.getPythonOutput()[1];
    }
}