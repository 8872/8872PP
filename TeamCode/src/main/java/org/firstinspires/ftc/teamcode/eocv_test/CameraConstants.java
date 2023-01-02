package org.firstinspires.ftc.teamcode.eocv_test;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Scalar;

public class CameraConstants {
    Scalar lowY = new Scalar(0, 50, 50);
    Scalar highY = new Scalar(255, 180, 95);
    Scalar lowR = new Scalar(0, 170, 0);
    Scalar highR = new Scalar(120, 255, 255);
    Scalar lowB = new Scalar(0, 0, 160);
    Scalar highB = new Scalar(100, 255, 255);
    public static double calibrationDistance = -1;
    public static double calibrationWidth = -1;
    public static double focalLength = -1;
    //Calib3d.getOptimalNewCameraMatrix();
}
