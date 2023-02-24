package org.firstinspires.ftc.teamcode.vision.opmodes;

import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.LinearFilter;
import org.firstinspires.ftc.teamcode.util.MedianFilter;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
@Config
public class BruhOpmode extends OpMode {

    int counter = 0;
    private AnalogInput turretEnc;
    private ServoEx armServo;
    private CRServoImplEx turretServo;
    private OpenCvCamera camera;
    JunctionWithArea pipeline;
    public static double pix_to_degree = -0.192;
    public static double targetPos = 51.1;
    public static double turretPos;
    public static double voltage = 0;
    public static double pos;
    public static double constantTarget = 300;
    MedianFilter medianFilter = new MedianFilter(100);
    LinearFilter lowpass = LinearFilter.singlePoleIIR(0.1, 0.02);
    ElapsedTime time = new ElapsedTime();
    @Override
    public void init() {

        turretServo = hardwareMap.get(CRServoImplEx.class, "turret");
        turretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turretEnc = hardwareMap.get(AnalogInput.class, "turretEnc");
        armServo = new SimpleServo(hardwareMap, "arm", 0, 355);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armServo.setPosition(0.85);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new JunctionWithArea();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320 , 180, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        time.reset();
    }

    @Override
    public void loop() {
        Rect rect;
        if((rect = pipeline.getRect()) != null) {
            double x = rect.x+((double) rect.width/2);
            telemetry.addData("center", x);
            telemetry.addData("error", (x-160));
            telemetry.addData("turret position", turretPosition());
            telemetry.addData("change", change(rect));
            telemetry.addData("target", constantTarget);
            telemetry.addData("pixDegreeConstant", pixDegreeConstant(rect));
            telemetry.addData("pos", pos);


        }else{
            telemetry.addData("nothing seen", true);
        }
        if(gamepad1.dpad_up){
            if(counter > 10) {
                Log.d("asd", "" + turretPosition());
                counter=0;
            }
            counter++;
        }
        if(gamepad1.x){
            turretServo.setPower(0.1);
        }
        else if(gamepad1.b){
            turretServo.setPower(-0.1);
        }
        else if(gamepad1.y){
            turretServo.setPower(-0.25);
        }
        else if(gamepad1.a){
            turretServo.setPower(0.25);
        }
        else{
            turretServo.setPower(0);
        }



        telemetry.update();
    }
    public double turretPosition(){
        double angle = (turretEnc.getVoltage() - 0.167) / 2.952 * 355;
        voltage = turretEnc.getVoltage();
        pos = -angle+355;
        turretPos = medianFilter.calculate(lowpass.calculate(pos));
        return turretPos;
    }

    public double change(Rect rect){
        double junctionX = rect.x+(double) rect.width/2;
        return ((junctionX-160)*pix_to_degree);
    }

    public double target(Rect rect){
        double junctionX = rect.x+(double) rect.width/2;
        return (turretPosition() + ((junctionX-160)*pix_to_degree));
    }


    public double pixDegreeConstant(Rect rect){
        double junctionX = rect.x+(double) rect.width/2;
        return (targetPos-turretPosition())/(junctionX-160);
    }
}
