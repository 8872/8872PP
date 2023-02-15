package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class JunctionPipeline extends OpenCvPipeline {
    private Point center = null;
    @Override
    public Mat processFrame(Mat input){

        //blur and convert to YCrCb color space
        Mat ycrcb = new Mat();
        Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        //thresholding values, may need to be further tuned
        Scalar lowThresh = new Scalar(0, 50, 10);
        Scalar highThresh = new Scalar(255, 180, 95);

        //thresholding for yellow objects
        Mat thresh = new Mat();
        Core.inRange(ycrcb, lowThresh, highThresh, thresh);

        //close to connect broken edges
        //scale everything by 5-6
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 15));
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, kernel);

        //find contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //draw contours in green
        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 2);

        Rect boundingRect;
        RotatedRect rotatedRect;
        RotatedRect largestRect = null;
        double largestWidth = 0;

        for (MatOfPoint contour : contours) {
            boundingRect = Imgproc.boundingRect(contour);

            //filter out small or disproportionate objects
            if(boundingRect.height < 200 || boundingRect.height / boundingRect.width <= 2)
                continue;

            //get minimum area bounding rectangle
            MatOfPoint2f  h = new MatOfPoint2f();
            contour.convertTo(h, CvType.CV_32F);
            rotatedRect = Imgproc.minAreaRect(h);

            //get rectangle with largest width
            if(rotatedRect.size.width > largestWidth) {
                largestWidth = rotatedRect.size.width;
                largestRect = rotatedRect.clone();
            }
        }

        //draw bounding rect
        if(largestRect != null) {
            Point[] rectPoints = new Point[4];
            largestRect.points(rectPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, rectPoints[i], rectPoints[(i + 1) % 4], new Scalar(255, 0, 0), 4);
            }
            center = largestRect.center.clone();
        }

        contours.clear();
        thresh.release();
        ycrcb.release();
        kernel.release();

        return input;
    }
    public Point getCenter() {
        return center;
    }
}
