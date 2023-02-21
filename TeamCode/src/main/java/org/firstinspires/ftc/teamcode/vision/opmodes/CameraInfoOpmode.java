package org.firstinspires.ftc.teamcode.vision.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.opencv.core.Rect;
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

    private Rect rect;

    @Override
    public void initialize() {
        super.initialize();

        turret.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 180, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        gb1(DPAD_DOWN).toggleWhenPressed(new InstantCommand(() -> turret.startTracking()),
                new InstantCommand(() -> turret.stopTracking()));

        gb1(A).whenPressed(new DownSequence(lift, turret, arm, claw));
        gb1(LEFT_BUMPER).toggleWhenPressed(claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 200)),
                claw.release().andThen(new ConditionalCommand(new DelayedCommand(arm.goTo(ArmSys.Pose.DOWN), 200),
                        new InstantCommand(), () -> (lift.getCurrentGoal() == Height.NONE.getHeight()))));

        // 180
        gb1(Y).whenPressed(new HighSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));
        gb1(X).whenPressed(new MediumSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));
        gb1(B).whenPressed(new LowSequence(lift, turret, arm, TurretSys.Pose.ONE_EIGHTY));

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
        lift.setDefaultCommand(lift.setPower(gamepadEx2::getRightY));
        drive.setDefaultCommand(drive.robotCentric(
                gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY));

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        rect = pipeline.getRect();
        if (rect != null) {
            int x = rect.x+rect.width/2;
            telemetry.addData("x", x);
            telemetry.addData("error", x-160);
        }

    }
}