package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class JunctionWithArea extends OpenCvPipeline {
    private Rect rect = null;
    @Override
    public Mat processFrame(Mat input){

        Imgproc.line(input, new Point(0,0), new Point(320, 0), new Scalar(0,0,0),50);

        //blur and convert to YCrCb color space
        Mat ycrcb = new Mat();
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        //thresholding values, may need to be further tuned
        Scalar lowThresh = new Scalar(0, 50, 10);
        Scalar highThresh = new Scalar(255, 180, 95);

        //thresholding for yellow objects
        Mat thresh = new Mat();
        Core.inRange(ycrcb, lowThresh, highThresh, thresh);


        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 25));
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_ERODE, kernel);


        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Mat biggestContour = null;
        double largestArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if(area > largestArea) {
                biggestContour = contour.clone();
                largestArea = area;
            }
        }

        if(biggestContour != null){
            rect = Imgproc.boundingRect(biggestContour);
            Imgproc.rectangle(input, rect, new Scalar(0,0,255), 1);
        }


        thresh.release();
        ycrcb.release();
        kernel.release();
        contours.clear();


        return input;
    }
    public Rect getRect() {
        return rect;
    }
}
