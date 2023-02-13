package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.command.group.HighSequence;
import org.firstinspires.ftc.teamcode.command.group.LowSequence;
import org.firstinspires.ftc.teamcode.command.group.MediumSequence;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.vision.JunctionDetection;
import org.openftc.easyopencv.OpenCvCamera;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

@Config
@TeleOp
public final class MainOpMode extends BaseOpMode {

    public static int goal = -100;

    public static final double PIX_TO_DEGREE = 22.0133;


    private OpenCvCamera camera;
    private JunctionDetection pipeline;

    @Override
    public void initialize() {
        super.initialize();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        pipeline = new JunctionDetection();
//        camera.setPipeline(pipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });


        gb1(LEFT_BUMPER).whileHeld(
                drive.slowMode(gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));
//        gb1(RIGHT_BUMPER).whileHeld(
//                drive.driveWithConeRotation(gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));


        gb2(DPAD_UP).whenPressed(lift.goTo(Height.HIGH));
        gb2(DPAD_LEFT).whenPressed(lift.goTo(Height.LOW));
        gb2(DPAD_RIGHT).whenPressed(lift.goTo(Height.MEDIUM));
        gb2(DPAD_DOWN).whenPressed(lift.goTo(Height.NONE));


        gb2(A).whenPressed(new DownSequence(lift, turret, arm));
        gb2(LEFT_BUMPER).toggleWhenPressed(claw.grab().andThen(arm.goTo(ArmSys.Pose.GRAB)),
                claw.release().andThen(arm.goTo(ArmSys.Pose.DOWN)));

        // 180
        gb2(Y).whenPressed(new HighSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));
        gb2(X).whenPressed(new MediumSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));
        gb2(B).whenPressed(new LowSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));

        gb1(GamepadKeys.Trigger.LEFT_TRIGGER).and(gb1(Y)).whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD));

        // forklift
//        gb2(RIGHT_BUMPER)
//                .and(gb2(Y))
//                .whenActive(lift.goTo(Height.HIGH).alongWith(arm.goTo(ArmSys.Pose.DOWN)));
//        gb2(RIGHT_BUMPER)
//                .and(gb2(X))
//                .whenActive(lift.goTo(Height.MEDIUM).alongWith(arm.goTo(ArmSys.Pose.DOWN)));
//        gb2(RIGHT_BUMPER)
//                .and(gb2(B))
//                .whenActive(lift.goTo(Height.LOW).alongWith(arm.goTo(ArmSys.Pose.DOWN)));
//
//        // back left
//        gb2(Y).whenPressed(new ConditionalCommand(
//                new HighSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD), new InstantCommand(),
//                triggerReaderLeft2::isDown));
//        gb2(X).whenPressed(new ConditionalCommand(
//                new MediumSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD), new InstantCommand(),
//                triggerReaderLeft2::isDown));
//        gb2(B).whenPressed(new ConditionalCommand(
//                new LowSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD), new InstantCommand(),
//                triggerReaderLeft2::isDown));


//        gb2(X).whenPressed(new UpSequence(lift, turret, arm, Height.HIGH, TurretSys.Pose.LEFT_FORWARD));
//        gb2(B).whenPressed(new UpSequence(lift, turret, arm, Height.HIGH, TurretSys.Pose.RIGHT));


        ;


        register(drive, lift, claw, turret, arm);
        drive.setDefaultCommand(drive.robotCentric(
                gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));

    }

//    @Override
//    public void run() {
//        super.run();
//
//        Point center = pipeline.uniqueCenter();
//        if (center != null) {
//            double error = (center.x - 640) / PIX_TO_DEGREE;
//            drive.setHeading(imu.getHeading() - error);
//        }
//    }
}