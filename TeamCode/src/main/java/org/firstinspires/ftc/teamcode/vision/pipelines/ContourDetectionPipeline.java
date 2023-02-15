package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ContourDetectionPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {

        //blur and convert to YCrCb color space
        Mat mat = new Mat();
        Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        //thresholding values, may need to be further tuned
        Scalar lowThresh = new Scalar(0, 50, 10);
        Scalar highThresh = new Scalar(255, 180, 95);

        //thresholding for yellow objects
        Mat thresh = new Mat();
        Core.inRange(mat, lowThresh, highThresh, thresh);

        //close to connect broken edges
        //scale everything by 5-6
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 2));
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, kernel);

        //find contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);








        return thresh;
    }
}
