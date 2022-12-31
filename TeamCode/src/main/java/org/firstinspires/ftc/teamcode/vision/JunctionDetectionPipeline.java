package org.firstinspires.ftc.teamcode.vision;

import com.google.ftcresearch.tfod.util.ImageUtils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class JunctionDetectionPipeline extends OpenCvPipeline {

    private final Scalar RED = new Scalar(255,0,0);
    private final Scalar GREEN = new Scalar(0,255,0);
    private final Scalar BLUE = new Scalar(0,0,255);
    private final Scalar BLACK = new Scalar(0,0,0);

    public static double low_Y = 0;
    public static double low_Cr = 50; //141
    public static double low_Cb = 50;
    public static double high_Y = 255;
    public static double high_Cr = 180; //230
    public static double high_Cb = 95;

    private final Scalar lowThresh = new Scalar(low_Y, low_Cr, low_Cb);
    private final Scalar highThresh = new Scalar(high_Y, high_Cr, high_Cb);

    @Override
    public Mat processFrame(Mat input) {
        // convert to ycrcb
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
        // blur
        Mat blur = new Mat();
        Imgproc.GaussianBlur(mat, blur, new Size(5.0, 5.0), 0);
        // thresh
        Mat thresh = new Mat();
        Core.inRange(blur, lowThresh, highThresh, thresh);

        // clean
        Size kernelSize = new Size(5, 5);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize);
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, kernel);

        // contour
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(thresh, contours, -1, GREEN, 3);
        double maxArea = Imgproc.contourArea(contours.get(0));
        Mat largestContour = new Mat();
        for (int i = 0; i < contours.size(); i++){
            if (Imgproc.contourArea(contours.get(i)) > maxArea){
                maxArea = Imgproc.contourArea(contours.get(i));
                largestContour = contours.get(i);
            }
        }

        Rect largestRect = Imgproc.boundingRect(largestContour);

        // filter largest area * (width/height)


        // find x of largest
        mat.release();
        blur.release();
        kernel.release();
        return thresh;
    }
}
