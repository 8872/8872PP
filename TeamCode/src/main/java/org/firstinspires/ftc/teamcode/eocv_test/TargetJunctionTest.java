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
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.GaussianBlur(thresh, thresh, new Size(5.0, 5.0), 0);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
        Imgproc.erode(thresh, thresh, kernel);


        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect boundRect;
        Rect largestRect = new Rect(0, 0, 0, 0);
        int largestIndex = -1;
        center = new Point();
        float[] radius = new float[1];
        ;
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            if(boundRect.height/boundRect.width >= 2 && boundRect.width > largestRect.width){
                largestRect = boundRect.clone();
                largestIndex = i;
            }

        }


        if(largestIndex >= 0){
            Imgproc.minEnclosingCircle(contoursPoly[largestIndex], center, radius);
            Imgproc.rectangle(input, largestRect.tl(), largestRect.br(), GREEN, 2);
            Imgproc.circle(input, center, (int) radius[0], RED, 2);
        }

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

