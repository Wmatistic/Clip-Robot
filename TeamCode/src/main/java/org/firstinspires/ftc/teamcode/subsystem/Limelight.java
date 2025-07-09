package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.CameraCalculations;
import org.firstinspires.ftc.teamcode.util.IntakeInverseKinematics;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Sample;

import java.util.ArrayList;
import java.util.List;
import java.lang.*;

public class Limelight {

    RobotHardware robot;
    public static LLResult result;

    public static ArrayList<Sample> samples;

    private int targetedSampleIndex;

    private int targetedSampleColor;

    public Limelight() {
        this.robot = RobotHardware.getInstance();
        result = robot.limelight.getLatestResult();

        samples = new ArrayList<>();
        targetedSampleIndex = 0;
    }

    public void refreshResult() {
        result = robot.limelight.getLatestResult();
    }

    public double[] getSampleLocations() {
        result = robot.limelight.getLatestResult();
        return result.getPythonOutput();
    }

    public void refreshSamples() {
        samples.clear();
        double[] cameraOutput = getSampleLocations();

        for (int i = 1; i < cameraOutput.length-1; i+=3) {
            if (cameraOutput[i] != 0 && cameraOutput[i + 1] != 0) {
                robot.cameraCalcs.SampleToRealWorld(cameraOutput[i], cameraOutput[i + 1]);

                double tempTurretAngle = IntakeInverseKinematics.getIKTurretAngle(CameraCalculations.worldPositionX, CameraCalculations.worldPositionY);
                double tempSlideExtension = IntakeInverseKinematics.getIKSlideExtension(CameraCalculations.worldPositionX, CameraCalculations.worldPositionY);

                if (!Double.isNaN(tempTurretAngle) && tempSlideExtension < (RobotConstants.Intake.slideMax + Intake.slideSampleCheck) && cameraOutput[i + 3] == targetedSampleColor){
                    samples.add(new Sample(CameraCalculations.worldPositionX, CameraCalculations.worldPositionY, cameraOutput[i + 2], cameraOutput[i + 3]));
                }
            }
        }
    }

    public Sample getTargetedSample() {
        return samples.get(targetedSampleIndex);
    }

    public void targetSampleColor(int color) {
        if (color >= 0 && color <= 2) {
            targetedSampleColor = color;
        }
    }

    public void targetNextSample() {
        if (targetedSampleIndex == samples.size()-1) {
            targetedSampleIndex = 0;
        } else {
            targetedSampleIndex++;
        }
    }

    public boolean hasSamples() {
        return !samples.isEmpty();
    }

    public ArrayList<Sample> getSamples() {
        return samples;
    }
}