package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Camera;

import com.arcrobotics.ftclib.command.Robot;
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
import java.util.Comparator;
import java.util.List;
import java.lang.*;

public class Limelight {

    RobotHardware robot;
    public static LLResult result;

    public static ArrayList<Sample> samples;

    private int targetedSampleIndex;

    private int targetedSampleColor;

    public enum SortingDirection {
        LEFT, RIGHT, NONE
    }

    private SortingDirection sortingDirection;

    public Limelight() {
        this.robot = RobotHardware.getInstance();
        result = robot.limelight.getLatestResult();

        samples = new ArrayList<>();
        targetedSampleIndex = 0;
        sortingDirection = SortingDirection.NONE;
    }

    public void refreshResult() {
        result = robot.limelight.getLatestResult();
    }

    public double[] getSampleLocations() {
        result = robot.limelight.getLatestResult();
        return result.getPythonOutput();
    }

    public void refreshSamples() {
        targetedSampleIndex = 0;
        IntakeInverseKinematics.reset();
        samples.clear();

        double[] sampleColorInput = {targetedSampleColor};
        robot.limelight.updatePythonInputs(sampleColorInput);

        double[] cameraOutput = getSampleLocations();

        for (int i = 1; i < cameraOutput.length-1; i+=4) {
            if (cameraOutput[i] != 0 && cameraOutput[i + 1] != 0) {
                robot.cameraCalcs.SampleToRealWorld(cameraOutput[i], cameraOutput[i + 1]);

                double tempTurretAngle = IntakeInverseKinematics.getIKTurretAngle(CameraCalculations.worldPositionX, CameraCalculations.worldPositionY);
                double tempSlideExtension = IntakeInverseKinematics.getIKSlideExtension(CameraCalculations.worldPositionX, CameraCalculations.worldPositionY);
                double tempSlideExtensionInches = IntakeInverseKinematics.getIKSlideExtensionInches(CameraCalculations.worldPositionX, CameraCalculations.worldPositionY);

                if (!Double.isNaN(tempTurretAngle) && !Double.isNaN(tempSlideExtension) && !Double.isNaN(tempSlideExtensionInches)){
                    if (tempSlideExtension < (RobotConstants.Intake.slideMax + robot.intake.getSlideSampleCheck()) && tempSlideExtension > -50 && cameraOutput[i + 3] == targetedSampleColor) {
                        samples.add(new Sample(CameraCalculations.worldPositionX, CameraCalculations.worldPositionY, cameraOutput[i + 2], cameraOutput[i + 3]));
                    }
                }
            }
        }

        if (sortingDirection == SortingDirection.LEFT) {
            samples.sort(Comparator.comparingDouble(Sample::getX).reversed());
        } else if (sortingDirection == SortingDirection.RIGHT) {
            samples.sort(Comparator.comparingDouble(Sample::getX));
        }

    }

    public void setSortingDirection(SortingDirection sortingDirection) {
        this.sortingDirection = sortingDirection;
    }

    public Sample getTargetedSample() {
        return samples.get(targetedSampleIndex);
    }

    public void targetSampleColor(int color) {
        if (color >= 0 && color <= 2) {
            targetedSampleColor = color;
        }
    }

    public int getTargetedSampleIndex() {
        return targetedSampleIndex;
    }

    public void setTargetedSampleIndex(int targetedSampleIndex) {
        this.targetedSampleIndex = targetedSampleIndex;
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