package org.firstinspires.ftc.teamcode.util;

import org.ejml.simple.SimpleMatrix;

public class CameraCalculations {
    double thetaXDeg, thetaYDeg;
    int imageWidth, imageHeight;
    double thetaX, thetaY;
    double fx, fy;
    SimpleMatrix K;
    int pixelX, pixelY;

    SimpleMatrix sampleCoords;
    SimpleMatrix worldPosition;

    public CameraCalculations() {
        thetaXDeg = 54.5;
        thetaYDeg = 42.0;
        imageWidth = 640;
        imageHeight = 480;

        thetaX = Math.toRadians(thetaXDeg);
        thetaY = Math.toRadians(thetaYDeg);

        fx = imageWidth / (2 * Math.tan(thetaX / 2));
        fy = imageHeight / (2 * Math.tan(thetaY / 2));

        K = new SimpleMatrix(3, 3);
        K.set(0, 0, fx);
        K.set(0, 2, (double) imageWidth / 2);
        K.set(1, 1, fy);
        K.set(1, 2, (double) imageHeight / 2);
        K.set(2, 2, 1);
    }
    public void SampleToRealWorld(int sampleX, int sampleY) {
        sampleCoords = new SimpleMatrix(3, 1);
        sampleCoords.set(0, 0, sampleX);
        sampleCoords.set(1, 0, pixelY);
        sampleCoords.set(2, 0, 1);

        SimpleMatrix normalizedCoords = K.invert().mult(sampleCoords);

        SimpleMatrix R = SimpleMatrix.identity(3);
        SimpleMatrix t = new SimpleMatrix(3, 1);
        t.set(0, 0, 0);
        t.set(1, 0, 0);
        t.set(2, 0, 100);

        worldPosition = R.mult(normalizedCoords).plus(t);
    }
}
