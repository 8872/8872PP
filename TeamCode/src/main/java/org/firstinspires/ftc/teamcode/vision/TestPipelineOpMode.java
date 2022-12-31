package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

@TeleOp(name = "test camera opmode")
public class TestPipelineOpMode extends OpMode {
    private OpenCvWebcam webcam1, webcam2;
    private JunctionDetectionPipeline junctionDetectionPipeline;
    private EmptyPipeline emptyPipeline;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Log.d("asd cameraMonitorViewId", ""+cameraMonitorViewId);
            int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            cameraMonitorViewId, //The container we're splitting
                            2, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally
            Log.d("asd view port ids", Arrays.toString(viewportContainerIds));

            webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
            webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

            Log.d("asd webcam 1 name", "" + hardwareMap.get(WebcamName.class, "Webcam 1").toString());
            Log.d("asd webcam 2 name", "" + hardwareMap.get(WebcamName.class, "Webcam 2").toString());

            junctionDetectionPipeline = new JunctionDetectionPipeline();
            emptyPipeline = new EmptyPipeline();

            webcam1.setPipeline(emptyPipeline);
            webcam2.setPipeline(junctionDetectionPipeline);


            webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam2.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });

    }

    @Override
    public void loop() {

    }
}
