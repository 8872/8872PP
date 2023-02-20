package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class JunctionWithWidth extends OpenCvPipeline {
    private RotatedRect rect = null;
    private boolean processed = false;
    //TODO: If the edges of the frame stop junctions touching them from being detected, add borders
    @Override
    public Mat processFrame(Mat input){

        //blur and convert to YCrCb color space
        Mat ycrcb = new Mat();
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        //thresholding values, may need to be further tuned
        Scalar lowThresh = new Scalar(0, 50, 10);
        Scalar highThresh = new Scalar(255, 180, 95);

        //thresholding for yellow objects
        Mat thresh = new Mat();
        Core.inRange(ycrcb, lowThresh, highThresh, thresh);




//        Mat masked = new Mat();
//        input.copyTo(masked, thresh);


        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 25));
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_ERODE, kernel);


        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //TODO: time optimization: remove once tuned
        //draw contours in green
        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 2);


        Rect boundingRect;
        RotatedRect rotatedRect;
        RotatedRect largestRect = null;
        double largestWidth = 0;

        for (MatOfPoint contour : contours) {
            boundingRect = Imgproc.boundingRect(contour);

            //filter out small or disproportionate objects
            if(boundingRect.height < 25 || boundingRect.height / boundingRect.width < 2)
                continue;

            //get minimum area bounding rectangle
            MatOfPoint2f  h = new MatOfPoint2f();
            contour.convertTo(h, CvType.CV_32F);
            rotatedRect = Imgproc.minAreaRect(h);

            //correct height and width values
            if(rotatedRect.size.width > rotatedRect.size.height){
                double temp = rotatedRect.size.width;
                rotatedRect.size.width = rotatedRect.size.height;
                rotatedRect.size.height = temp;
            }

            //get rectangle with largest width
            if(rotatedRect.size.width > largestWidth) {
                largestWidth = rotatedRect.size.width;
                largestRect = rotatedRect.clone();
            }
            h.release();
        }

        //ensure a junction was detected
        if(largestRect != null) {
            //TODO: time optimization: remove once tuned
            //draw bounding rect in red
            Point[] rectPoints = new Point[4];
            largestRect.points(rectPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, rectPoints[i], rectPoints[(i + 1) % 4], new Scalar(255, 0, 0), 4);
            }

            rect = largestRect.clone();
            processed = true;
        }

        thresh.release();
        ycrcb.release();
        kernel.release();
        contours.clear();


        return input;
    }
    public RotatedRect getRect() {
        if(!processed) return null;
        processed = false;
        return rect;
    }
}
