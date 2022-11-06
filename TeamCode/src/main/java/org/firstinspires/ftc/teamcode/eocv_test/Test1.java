package org.firstinspires.ftc.teamcode.eocv_test;


import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class Test1 extends OpenCvPipeline {

    Point[] centers;
    public static boolean showThresh = false;

    @Override
    public Mat processFrame(Mat input) {

        Scalar RED = new Scalar(255,0,0);
        Scalar GREEN = new Scalar(0,255,0);
        Scalar BLUE = new Scalar(0,0,255);

        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowThresh = new Scalar(0.0, 141.0, 0.0);
        Scalar highThresh = new Scalar(255.0, 230.0, 95.0);
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
            Imgproc.drawContours(input, contoursPolyList, i, BLUE);
            Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), GREEN, 2);
            Imgproc.circle(input, centers[i], (int) radius[i][0], RED, 2);
        }

        mat.release();
        //thresh.release();
        kernel.release();
        edges.release();
        hierarchy.release();
//        input.push_back(thresh);
        if(showThresh) {
            input.release();
            return thresh;
        }
        else {
            thresh.release();
            return input;
        }
    }

    public Point[] getCenters(){
        return centers;
    }

}

