package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class JunctionWithArea extends OpenCvPipeline {
    private Rect rect = null;
    private Rect ratioCheck;
    private final Scalar lowThresh = new Scalar(0, 50, 10);
    private final Scalar highThresh = new Scalar(255, 180, 95);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 25));
    private final ArrayList<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input){
        //blur and convert to YCrCb color space
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);

        //thresholding values, may need to be further tuned

        //thresholding for yellow objects

        Core.inRange(input, lowThresh, highThresh, input);

        Imgproc.morphologyEx(input, input, Imgproc.MORPH_ERODE, kernel);

        Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Mat biggestContour = null;
        double largestArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            ratioCheck = Imgproc.boundingRect(contour);

            if(ratioCheck.height / ratioCheck.width >= 1 && area > largestArea) {
                biggestContour = contour;
                largestArea = area;
            }
        }

        if(biggestContour != null){
            rect = Imgproc.boundingRect(biggestContour);
            biggestContour.release();
        }

        kernel.release();
        contours.clear();

        return input;
    }
    public Rect getRect() {
        return rect;
    }
}
