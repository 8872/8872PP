package org.firstinspires.ftc.teamcode.eocv_test;


import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class ThreshPipeline extends OpenCvPipeline {
    Point[] centers;
    public static boolean showThresh = true;
    public static boolean red = true;
    public static boolean blue = false;
    public static double low_Y = 0;
    public static double low_Cr = 50; //141
    public static double low_Cb = 50;
    public static double high_Y = 255;
    public static double high_Cr = 180; //230
    public static double high_Cb = 95;


    @Override
    public Mat processFrame(Mat input) {

        Scalar RED = new Scalar(255,0,0);
        Scalar GREEN = new Scalar(0,255,0);
        Scalar BLUE = new Scalar(0,0,255);

        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        if(red){
            high_Y = 120;
            low_Cr = 170;
            low_Cb = 0;
            high_Cr = 255;
            high_Cb = 255;
        }else if(blue){
            high_Y = 100;
            low_Cr = 0;
            low_Cb = 160;
            high_Cr = 255;
            high_Cb = 255;
        }

        Scalar lowThresh = new Scalar(low_Y, low_Cr, low_Cb);
        Scalar highThresh = new Scalar(high_Y, high_Cr, high_Cb);

        Mat thresh = new Mat();

        Core.inRange(mat, lowThresh, highThresh, thresh);
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.GaussianBlur(thresh, thresh, new Size(5.0, 5.0), 0);


        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }


        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(thresh, contoursPolyList, i, BLUE);
            Imgproc.rectangle(thresh, boundRect[i].tl(), boundRect[i].br(), GREEN, 2);
            Imgproc.circle(thresh, centers[i], (int) radius[i][0], RED, 2);
        }

        mat.release();
        edges.release();
        hierarchy.release();
        if(showThresh) {
            return thresh;
        } else {
            thresh.release();
            return input;
        }
    }
}