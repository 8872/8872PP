package org.firstinspires.ftc.teamcode.eocv_test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class Test extends OpMode {

    private OpenCvCamera camera;
    JunctionDetection pipeline;
    private RevIMU imu;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        imu = new RevIMU(hardwareMap);
        imu.init();
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new JunctionDetection();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280 , 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }

    @Override
    public void loop() {

        Point center = pipeline.getCenter();

        if(center!=null) {
            double x = center.x;
            double y = center.y;
            telemetry.addData("error", (x-640) + ", " + (y-360));
            telemetry.addData("center", x + ", " + y);
            telemetry.addData("heading", imu.getHeading());
        }
        telemetry.addData("width", pipeline.getWidth());
        telemetry.addData("height", pipeline.getHeight());
        telemetry.update();
    }
}
