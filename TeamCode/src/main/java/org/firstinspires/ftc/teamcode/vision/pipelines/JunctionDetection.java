package org.firstinspires.ftc.teamcode.vision.pipelines;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled
public class JunctionDetection extends OpenCvPipeline {

    boolean newResultFlag = false;
    Point center;
    Point finalCenter = new Point();
    double finalWidth = -1;
    double finalHeight = -1;
    public static boolean threshThing = false;
    @Override
    public Mat processFrame(Mat input) {

        Scalar RED = new Scalar(255,0,0);
        Scalar GREEN = new Scalar(0,255,0);
        Scalar BLUE = new Scalar(0,0,255);
        Scalar BLACK = new Scalar(0,0,0);

        Imgproc.line(input, new Point(0,0), new Point(1280, 0),BLACK,4);
        Imgproc.line(input, new Point(0,720), new Point(1280, 720),BLACK,4);

        Mat mat = new Mat();
        Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowThresh = new Scalar(0, 50, 50);
        Scalar highThresh = new Scalar(255, 180, 95);
        Mat thresh = new Mat();

        Core.inRange(mat, lowThresh, highThresh, thresh);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        Imgproc.erode(thresh, thresh, kernel);
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, new Mat());


        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);


        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        RotatedRect boundRect;
        RotatedRect largestRect = new RotatedRect();
        double largestWidth = -1;
        double largestHeight = -1;
        int largestIndex = -1;
        center = new Point();
        float[] radius = new float[1];
        Point[] rectanglePoints = new Point[4];
        Rect regularRect;

        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect = Imgproc.minAreaRect(new MatOfPoint2f(contoursPoly[i].toArray()));

            boundRect.points(rectanglePoints);
            double width = Math.sqrt(Math.pow(rectanglePoints[2].x-rectanglePoints[1].x, 2)+
                    Math.pow(rectanglePoints[2].y-rectanglePoints[1].y, 2));
            double height = Math.sqrt(Math.pow(rectanglePoints[0].x-rectanglePoints[1].x, 2)+
                    Math.pow(rectanglePoints[0].y-rectanglePoints[1].y, 2));

            regularRect = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));


            if(regularRect.height < 540) continue;
            if(regularRect.height/regularRect.width>= 2){
                if(height<width){
                    double temp = width;
                    width = height;
                    height = temp;
                }
                if(width > largestWidth){
                    largestWidth = width;
                    largestHeight = height;
                    largestRect = boundRect.clone();
                    largestIndex = i;
                }
            }
            finalWidth = largestWidth;
            finalHeight = largestHeight;
        }

        if(largestIndex >= 0){
            Imgproc.minEnclosingCircle(contoursPoly[largestIndex], center, radius);
            largestRect.points(rectanglePoints);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(input, rectanglePoints[j], rectanglePoints[(j+1) % 4], GREEN, 3);
            }
            Imgproc.circle(input, center, (int) radius[0], RED, 2);
        }
        finalCenter = center.clone();
        newResultFlag = true;

        Imgproc.circle(input, new Point(640, 360), 8, BLUE, 8);

        mat.release();
        edges.release();
        hierarchy.release();

        if(threshThing){
            return thresh;
        }else{
            thresh.release();
            return input;
        }
    }
    public Point getCenter() {
        return finalCenter;
    }
    public Point uniqueCenter(){
        if(newResultFlag) return finalCenter;
        return null;
    }
    public double getWidth(){
        return finalWidth;
    }
    public double getHeight(){
        return finalHeight;
    }
}