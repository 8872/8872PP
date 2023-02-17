package org.firstinspires.ftc.teamcode.vision.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.command.group.HighSequence;
import org.firstinspires.ftc.teamcode.command.group.LowSequence;
import org.firstinspires.ftc.teamcode.command.group.MediumSequence;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;
import org.firstinspires.ftc.teamcode.vision.pipelines.InfoPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionDetection;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Objects;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

@Config
@TeleOp
public final class BruhOpmode extends BaseOpMode {

    private OpenCvCamera camera;
    private InfoPipeline pipeline;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);

        initHardware();
        setUpHardwareDevices();

        imu = new RevIMU(hardwareMap);
        imu.init();

        drive = new DriveSys(fL, fR, bL, bR, imu);
        lift = new LiftSys(dr4bLeftMotor, dr4bRightMotor, limitSwitch);
        claw = new ClawSys(clawServo);
        arm = new ArmSys(armServo);
        flipper = new FlipperSys(flipperServo);

        rrDrive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new InfoPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        gb1(DPAD_UP).whenPressed(lift.goTo(Height.HIGH));
        gb1(DPAD_LEFT).whenPressed(lift.goTo(Height.LOW));
        gb1(DPAD_RIGHT).whenPressed(lift.goTo(Height.MEDIUM));
        gb1(DPAD_DOWN).whenPressed(lift.goTo(Height.NONE));

        gb1(LEFT_BUMPER).toggleWhenPressed(claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 200)),
                claw.release().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.DOWN), 200)));


        drive.setDefaultCommand(drive.robotCentric(
                gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));

    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        RotatedRect rect = pipeline.getRect();
        if (rect != null) {
            telemetry.addData("center x", rect.center.x);
            telemetry.addData("center y", rect.center.y);
            telemetry.addData("height", rect.size.height);
            telemetry.addData("height width ratio", rect.size.height / rect.size.width);
            telemetry.addData("fps", camera.getFps());
            telemetry.update();
            if(gamepad1.b){
                turretServo.rotateByAngle(-(rect.center.x-640)/30);
            }
        }

    }

//    BufferedWriter x = new BufferedWriter(new FileWriter("src/main/java/org/firstinspires/ftc/teamcode/vision/data/x"));
//    BufferedWriter y = new BufferedWriter(new FileWriter("src/main/java/org/firstinspires/ftc/teamcode/vision/data/y"));
//    BufferedWriter height = new BufferedWriter(new FileWriter("src/main/java/org/firstinspires/ftc/teamcode/vision/data/height"));
//    BufferedWriter heightWidthRatio = new BufferedWriter(new FileWriter("src/main/java/org/firstinspires/ftc/teamcode/vision/data/heightWidthRatio"));
//    BufferedWriter horizontalChange = new BufferedWriter(new FileWriter("src/main/java/org/firstinspires/ftc/teamcode/vision/data/horizontalChange"));
//    BufferedWriter heightChange = new BufferedWriter(new FileWriter("src/main/java/org/firstinspires/ftc/teamcode/vision/data/heightChange"));
//    BufferedWriter driveVelocity = new BufferedWriter(new FileWriter("src/main/java/org/firstinspires/ftc/teamcode/vision/data/driveVelocity"));
//    BufferedWriter fps = new BufferedWriter(new FileWriter("src/main/java/org/firstinspires/ftc/teamcode/vision/data/fps"));
//    @Override
//    public void run() {
//        if(gamepad1.back){
////            try {
//                RotatedRect rect = pipeline.getRect();
//                if (rect != null) {
//                    telemetry.addData("center x", rect.center.x);
////                    x.write(rect.center.x + "\n");
//                    telemetry.addData("center y", rect.center.y);
////                    y.write(rect.center.y+"\n");
//                    telemetry.addData("height", rect.size.height);
////                    height.write(rect.size.height+"\n");
//                    telemetry.addData("height width ratio", rect.size.height/rect.size.width);
////                    heightWidthRatio.write(rect.size.height/rect.size.width+"\n");
//                    telemetry.addData("horizontal change", rect.center.x-previousX);
////                    horizontalChange.write(rect.center.x-previousX+"\n");
//                    telemetry.addData("height change", rect.size.height-previousHeight);
////                    heightChange.write("rect.size.height-previousHeight"+"\n");
//                    previousX = rect.center.x;
//                    previousHeight=rect.size.height;
//
////                    Pose2d poseVelo = Objects.requireNonNull(roadrunnerDrive.getPoseVelocity(), "poseVelocity() was null in CameraInfoOpmode");
////                    telemetry.addData("drive velocity", Math.sqrt(Math.pow(poseVelo.getX(), 2) + Math.pow(poseVelo.getY(), 2)));
////                    driveVelocity.write(Math.sqrt(Math.pow(poseVelo.getX(), 2) + Math.pow(poseVelo.getY(), 2))+"\n");
//                    telemetry.addData("fps", camera.getFps());
////                    fps.write(camera.getFps()+"\n");
//                    telemetry.update();
//                }
////            } catch (IOException e) {
////                throw new RuntimeException(e);
////            }
//        }
//
//
//
//
//    }
}