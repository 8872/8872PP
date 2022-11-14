package org.firstinspires.ftc.teamcode.eocv_test;


import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class TargetJunctionTest extends OpenCvPipeline {

    Point center;

    @Override
    public Mat processFrame(Mat input) {

        Scalar RED = new Scalar(255,0,0);
        Scalar GREEN = new Scalar(0,255,0);
        Scalar BLUE = new Scalar(0,0,255);

        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowThresh = new Scalar(JunctionDetectionTest.low_Y, JunctionDetectionTest.low_Cr, JunctionDetectionTest.low_Cb);
        Scalar highThresh = new Scalar(JunctionDetectionTest.high_Y, JunctionDetectionTest.high_Cr, JunctionDetectionTest.high_Cb);
        Mat thresh = new Mat();

        Core.inRange(mat, lowThresh, highThresh, thresh);
        //Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        //Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.GaussianBlur(thresh, thresh, new Size(5.0, 5.0), 0);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
        //Imgproc.erode(thresh, thresh, kernel);


        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        RotatedRect boundRect;
        RotatedRect largestRect = new RotatedRect();
        double largestWidth = 0;
        int largestIndex = -1;
        center = new Point();
        float[] radius = new float[1];
        Point[] rectanglePoints = new Point[4];

        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect = Imgproc.minAreaRect(new MatOfPoint2f(contoursPoly[i].toArray()));

            boundRect.points(rectanglePoints);
            double width = Math.sqrt(Math.pow(rectanglePoints[2].x-rectanglePoints[1].x, 2)+
                    Math.pow(rectanglePoints[2].y-rectanglePoints[1].y, 2));
            double height = Math.sqrt(Math.pow(rectanglePoints[0].x-rectanglePoints[1].x, 2)+
                    Math.pow(rectanglePoints[0].y-rectanglePoints[1].y, 2));

            if(height/width >= 2 && width > largestWidth){
                largestWidth = width;
                largestRect = boundRect.clone();
                largestIndex = i;
            }

        }

        if(largestIndex >= 0){
            Imgproc.minEnclosingCircle(contoursPoly[largestIndex], center, radius);
            largestRect.points(rectanglePoints);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(input, rectanglePoints[j], rectanglePoints[(j+1) % 4], GREEN, 3);
            }
            Imgproc.circle(input, center, (int) radius[0], RED, 2);
        }

        Imgproc.circle(input, new Point(640, 360), 8, BLUE, 8);

        mat.release();
        kernel.release();
        edges.release();
        hierarchy.release();
        thresh.release();
        return input;
    }
    public Point getCenter(){
        return center;
    }
}