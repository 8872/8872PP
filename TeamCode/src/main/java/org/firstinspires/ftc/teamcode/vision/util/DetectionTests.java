package org.firstinspires.ftc.teamcode.vision.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Objects;
import java.util.Queue;

public class DetectionTests {
    private SampleMecanumDrive drive;
    //TODO: tune constants
    public static final double MIN_CONTOUR_AREA = 250;
    public static final int HEIGHT_RATIO = 2;
    public static final double RECT_PROPORTION = 0.8;
    public static final double APPROX_PX_TO_IN = 0;
    public static final double CHANGE_SCALAR = 2;
    public static final double MAX_PATTERN_SHIFT = 300;

    public DetectionTests(HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
    }

    //could be replaced with height test
    public static boolean minAreaTest(MatOfPoint contour){
        return Imgproc.contourArea(contour) > MIN_CONTOUR_AREA;
    }

    public static boolean ratioTest(MatOfPoint contour){
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.height / boundingRect.width >= HEIGHT_RATIO;
    }

    public static boolean rectPropTest(RotatedRect minRect, MatOfPoint contour){
        return Imgproc.contourArea(contour) / minRect.size.area() <= RECT_PROPORTION;
    }

    public boolean velocityTest(int pxChange, double fps){
        Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() was null in vision DetectionTests");
        double currentVelo = Math.sqrt(Math.pow(poseVelo.getX(), 2) + Math.pow(poseVelo.getY(), 2));
        return ((pxChange/fps)*APPROX_PX_TO_IN*CHANGE_SCALAR < currentVelo);
    }

    public boolean patternTest(LinkedList<Double> pxChanges, double currentPxChange){
        double total = 0;
        for(int i = 1; i <= pxChanges.size(); i++){
            total += i*pxChanges.get(i-1);
        }
        total /= ((double) pxChanges.size() * (pxChanges.size() + 1) /2);
        return Math.abs(currentPxChange-total) < MAX_PATTERN_SHIFT;
    }

    public static void minAreaFilter(ArrayList<MatOfPoint> contours){
        contours.removeIf(c -> Imgproc.contourArea(c) > MIN_CONTOUR_AREA);
    }

    public static void ratioFilter(ArrayList<MatOfPoint> contours){
        contours.removeIf(c -> Imgproc.boundingRect(c).height / Imgproc.boundingRect(c).width >= HEIGHT_RATIO);
    }

}
