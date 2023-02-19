package org.firstinspires.ftc.teamcode.vision.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import fi.iki.elonen.NanoHTTPD;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.command.group.HighSequence;
import org.firstinspires.ftc.teamcode.command.group.LowSequence;
import org.firstinspires.ftc.teamcode.command.group.MediumSequence;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.vision.pipelines.InfoPipeline;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

@Config
@TeleOp
public final class CameraInfoOpmode extends BaseOpMode {

    private RotatedRect rect;

    // NOTE: If you're using NanoHTTPD >= 3.0.0 the namespace is different,
    //       instead of the above import use the following:
    // import org.nanohttpd.NanoHTTPD;

    public class App extends NanoHTTPD {
        private DoubleSupplier fps;

        public App(DoubleSupplier fps) throws IOException {
            super(7070);
            start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
            System.out.println("\nRunning! Point your browsers to http://localhost:7070/ \n");
            this.fps = fps;
        }

        public App() throws IOException {
            super(7070);
            start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
            System.out.println("\nRunning! Point your browsers to http://localhost:7070/ \n");
        }

        @Override
        public Response serve(IHTTPSession session) {
            String msg = "<html><body><h1>Hello server</h1>\n";

            msg += "fps: " + fps.getAsDouble() + "\n";

            return newFixedLengthResponse(msg + "</body></html>\n");
        }
    }


    double previousHeight = 0;
    double previousX = 0;
//    SampleMecanumDrive roadrunnerDrive = new SampleMecanumDrive(hardwareMap);

    public static int goal = -100;

    public static final double PIX_TO_DEGREE = 22.0133;

    private OpenCvCamera camera;
    private InfoPipeline pipeline;

    @Override
    public void initialize() {
        super.initialize();


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

        try {
            new App(camera::getFps);
        } catch (IOException ioe) {
            System.err.println("Couldn't start server:\n" + ioe);
        }


//        gb1(LEFT_BUMPER).whileHeld(
//                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));
//        gb1(RIGHT_BUMPER).whileHeld(
//                drive.driveWithConeRotation(gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));


        gb1(DPAD_UP).whenPressed(lift.goTo(Height.HIGH));
        gb1(DPAD_LEFT).whenPressed(lift.goTo(Height.LOW));
        gb1(DPAD_RIGHT).whenPressed(lift.goTo(Height.MEDIUM));
        gb1(DPAD_DOWN).whenPressed(lift.goTo(Height.NONE));


        gb1(A).whenPressed(new DownSequence(lift, turret, arm));
        gb1(LEFT_BUMPER).toggleWhenPressed(claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 200)),
                claw.release().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.DOWN), 200)));

        // 180
        gb1(Y).whenPressed(new HighSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));
        gb1(X).whenPressed(new MediumSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));
//        gb1(B).whenPressed(new LowSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));

//         forklift
        gb1(RIGHT_BUMPER)
                .and(gb1(Y))
                .whenActive(lift.goTo(Height.HIGH).alongWith(arm.goTo(ArmSys.Pose.HORIZONTAL)));
        gb1(RIGHT_BUMPER)
                .and(gb1(X))
                .whenActive(lift.goTo(Height.MEDIUM).alongWith(arm.goTo(ArmSys.Pose.HORIZONTAL)));

        // front left
        gb1(LEFT_TRIGGER).and(gb1(Y))
                .whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD));
        gb1(LEFT_TRIGGER).and(gb1(X))
                .whenActive(new MediumSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD));
        gb1(LEFT_TRIGGER).and(gb1(B))
                .whenActive(new LowSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD));

        // front right
        gb1(RIGHT_TRIGGER).and(gb1(Y))
                .whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.RIGHT_FORWARD));
        gb1(RIGHT_TRIGGER).and(gb1(X))
                .whenActive(new MediumSequence(lift, turret, arm, TurretSys.Pose.RIGHT_FORWARD));
        gb1(RIGHT_TRIGGER).and(gb1(B))
                .whenActive(new LowSequence(lift, turret, arm, TurretSys.Pose.RIGHT_FORWARD));

        // back left
        gb1(DPAD_LEFT).and(gb1(Y))
                .whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.LEFT_BACK));
        gb1(DPAD_LEFT).and(gb1(X))
                .whenActive(new MediumSequence(lift, turret, arm, TurretSys.Pose.LEFT_BACK));
        gb1(DPAD_LEFT).and(gb1(B))
                .whenActive(new LowSequence(lift, turret, arm, TurretSys.Pose.LEFT_BACK));

        // back right
        gb1(DPAD_RIGHT).and(gb1(Y))
                .whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.RIGHT_BACK));
        gb1(DPAD_RIGHT).and(gb1(X))
                .whenActive(new MediumSequence(lift, turret, arm, TurretSys.Pose.RIGHT_BACK));
        gb1(DPAD_RIGHT).and(gb1(B))
                .whenActive(new LowSequence(lift, turret, arm, TurretSys.Pose.RIGHT_BACK));


        register(drive, lift, claw, turret, arm);
        drive.setDefaultCommand(drive.robotCentric(
                gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        rect = pipeline.getRect();
        if (rect != null) {
            telemetry.addData("center x", rect.center.x);
            telemetry.addData("center y", rect.center.y);
            telemetry.addData("height", rect.size.height);
            telemetry.addData("height width ratio", rect.size.height / rect.size.width);
            telemetry.addData("horizontal change", rect.center.x - previousX);
            telemetry.addData("height change", rect.size.height - previousHeight);
            telemetry.addData("fps", camera.getFps());
            telemetry.update();
            if (gamepad1.b) {
                turretServo.rotateByAngle((rect.center.x - 640) / 30);
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