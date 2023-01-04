package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.drive.*;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "another rotation test")
public final class CameraRotate extends BaseOpMode {
    private DriveWithJunctionRotation driveWithJunctionRotation;
    private OpenCvCamera camera;
    JunctionDetection pipeline;
    private RevIMU imu;

    public final double PIX_TO_DEGREE = 22.0133;
        private DefaultRobotCentricDrive robotCentricDrive;


    @Override
    public void initialize() {
        super.initialize();

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

//        robotCentricDrive = new DefaultRobotCentricDrive(drive, gamepadEx1::getLeftX,
//                gamepadEx1::getRightX, gamepadEx1::getLeftY);


        driveWithJunctionRotation = new DriveWithJunctionRotation(drive, gamepadEx1::getLeftX,
                gamepadEx1::getRightX, gamepadEx1::getLeftY);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SetHeading(drive, 0));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetHeading(drive, 180));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetHeading(drive, -90));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetHeading(drive, 90));

        register(drive);
        drive.setDefaultCommand(driveWithJunctionRotation);
    }

    @Override
    public void run(){
        super.run();

        Point center = pipeline.uniqueCenter();
        if(gamepad1.b){
            if(center != null){
                double error = (center.x-640)/PIX_TO_DEGREE;
                drive.setHeading(imu.getHeading()-error);
            }
        }
    }
}