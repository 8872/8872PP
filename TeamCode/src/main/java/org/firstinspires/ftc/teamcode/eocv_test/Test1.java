package org.firstinspires.ftc.teamcode.eocv_test;

import static org.opencv.core.Core.inRange;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Test1 extends OpenCvPipeline {
    Mat YCrCb = new Mat();

    static final Scalar GREEN = new Scalar(0, 255, 0);

    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    Scalar lowerThreshold = new Scalar(0.0, 141.0, 0.0);
    Scalar upperThreshold = new Scalar(255.0, 230.0, 95.0);

    Mat filtered = new Mat();

    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

    MatOfPoint largest = new MatOfPoint();

    Rect rect = new Rect();

    Telemetry telemetry;
    public Test1(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(YCrCb, YCrCb, kernel);

        inRange(YCrCb, lowerThreshold, upperThreshold, filtered);

        Imgproc.findContours(filtered, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        largest = contours.get(0);
        int largestIndex = 0;
        for(int i = 0; i < contours.size(); i++){
            if(Imgproc.contourArea(contours.get(i)) > Imgproc.contourArea(largest)) {
                largest = contours.get(i);
                largestIndex = i;
            }
        }
        Imgproc.drawContours(filtered, contours, largestIndex, GREEN);
        rect = Imgproc.boundingRect(largest);

        telemetry.addData("Red Contour ", "%7d,%7d", rect.x + (rect.width/2), rect.y + (rect.height/2));


        YCrCb.release();
        telemetry.update();

        return filtered;

    }
}

