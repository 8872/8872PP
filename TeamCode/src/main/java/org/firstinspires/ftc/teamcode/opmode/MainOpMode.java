package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.command.group.GrabAndLift;
import org.firstinspires.ftc.teamcode.util.Height;
import org.firstinspires.ftc.teamcode.vision.JunctionDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

@Config
@TeleOp(name = "Main TeleOp")
public final class MainOpMode extends BaseOpMode {

    public static int goal = -100;

    public static final double PIX_TO_DEGREE = 22.0133;


    private OpenCvCamera camera;
    private JunctionDetection pipeline;

    @Override
    public void initialize() {
        super.initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
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


        gb1(LEFT_BUMPER).whileHeld(
                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));
        gb1(RIGHT_BUMPER).whileHeld(
                drive.driveWithConeRotation(gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));

        gb2(LEFT_BUMPER).toggleWhenPressed(new GrabAndLift(lift, claw), claw.runReleaseCommand());

        gb2(LEFT_STICK_BUTTON).toggleWhenPressed(slide.out(), slide.in());

//        gb2(Y).whenPressed(new LiftUp(lift, slide, Height.HIGH));
//        gb2(X).whenPressed(new LiftUp(lift, slide, Height.MEDIUM));
//        gb2(B).whenPressed(new LiftUp(lift, slide, Height.LOW));
//        gb2(A).whenPressed(new LiftDown(lift, slide, claw));

        gb2(DPAD_UP).whenPressed(lift.goTo(Height.FIRST));
        gb2(DPAD_LEFT).whenPressed(lift.goTo(Height.SECOND));
        gb2(DPAD_RIGHT).whenPressed(lift.goTo(Height.THIRD));
        gb2(DPAD_DOWN).whenPressed(lift.goTo(Height.FOURTH));

        //forklift
        gb2(RIGHT_BUMPER)
                .and(gb2(A))
                .whenActive(lift.goTo(Height.NONE));
        gb2(RIGHT_BUMPER)
                .and(gb2(B))
                .whenActive(lift.goTo(Height.LOW));
        gb2(RIGHT_BUMPER)
                .and(gb2(X))
                .whenActive(lift.goTo(Height.MEDIUM));
        gb2(RIGHT_BUMPER)
                .and(gb2(Y))
                .whenActive(lift.goTo(Height.HIGH));


        register(drive, lift, claw, slide);
        drive.setDefaultCommand(drive.robotCentric(
                gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));

    }

    @Override
    public void run() {
        super.run();

        Point center = pipeline.uniqueCenter();
        if(center != null){
            double error = (center.x-640)/PIX_TO_DEGREE;
            drive.setHeading(imu.getHeading()-error);
        }
    }
}