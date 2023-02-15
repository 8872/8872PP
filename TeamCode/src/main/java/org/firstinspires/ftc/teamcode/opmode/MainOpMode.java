package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.command.group.HighSequence;
import org.firstinspires.ftc.teamcode.command.group.LowSequence;
import org.firstinspires.ftc.teamcode.command.group.MediumSequence;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.vision.JunctionDetection;
import org.openftc.easyopencv.OpenCvCamera;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

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
        gb2(LEFT_BUMPER).toggleWhenPressed(claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 200)),
                claw.release().andThen(new ConditionalCommand(new DelayedCommand(arm.goTo(ArmSys.Pose.DOWN), 200),
                        new InstantCommand(), () -> (lift.getCurrentGoal() == Height.NONE.getHeight()))));

        // 180
        gb2(Y).whenPressed(new HighSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));
        gb2(X).whenPressed(new MediumSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));
        gb2(B).whenPressed(new LowSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));

//         forklift
        gb2(RIGHT_BUMPER)
                .and(gb2(Y))
                .whenActive(lift.goTo(Height.HIGH).alongWith(arm.goTo(ArmSys.Pose.FORKLIFT)));
        gb2(RIGHT_BUMPER)
                .and(gb2(X))
                .whenActive(lift.goTo(Height.MEDIUM).alongWith(arm.goTo(ArmSys.Pose.FORKLIFT)));

        // front left
        gb2(LEFT_TRIGGER).and(gb2(Y))
                .whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD));
        gb2(LEFT_TRIGGER).and(gb2(X))
                .whenActive(new MediumSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD));
        gb2(LEFT_TRIGGER).and(gb2(B))
                .whenActive(new LowSequence(lift, turret, arm, TurretSys.Pose.LEFT_FORWARD));

        // front right
        gb2(RIGHT_TRIGGER).and(gb2(Y))
                .whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.RIGHT_FORWARD));
        gb2(RIGHT_TRIGGER).and(gb2(X))
                .whenActive(new MediumSequence(lift, turret, arm, TurretSys.Pose.RIGHT_FORWARD));
        gb2(RIGHT_TRIGGER).and(gb2(B))
                .whenActive(new LowSequence(lift, turret, arm, TurretSys.Pose.RIGHT_FORWARD));

        // back left
        gb2(DPAD_LEFT).and(gb2(Y))
                .whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.LEFT_BACK));
        gb2(DPAD_LEFT).and(gb2(X))
                .whenActive(new MediumSequence(lift, turret, arm, TurretSys.Pose.LEFT_BACK));
        gb2(DPAD_LEFT).and(gb2(B))
                .whenActive(new LowSequence(lift, turret, arm, TurretSys.Pose.LEFT_BACK));

        // back right
        gb2(DPAD_RIGHT).and(gb2(Y))
                .whenActive(new HighSequence(lift, turret, arm, TurretSys.Pose.RIGHT_BACK));
        gb2(DPAD_RIGHT).and(gb2(X))
                .whenActive(new MediumSequence(lift, turret, arm, TurretSys.Pose.RIGHT_BACK));
        gb2(DPAD_RIGHT).and(gb2(B))
                .whenActive(new LowSequence(lift, turret, arm, TurretSys.Pose.RIGHT_BACK));


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