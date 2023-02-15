package org.firstinspires.ftc.teamcode.vision.pipelines;


import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ThreshPipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {


        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowThresh = new Scalar(0, 50, 10);
        Scalar highThresh = new Scalar(255, 180, 95);
        Mat thresh = new Mat();

        Core.inRange(mat, lowThresh, highThresh, thresh);

        return thresh;
    }
}