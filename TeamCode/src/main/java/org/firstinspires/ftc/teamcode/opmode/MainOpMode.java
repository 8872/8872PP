package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.drive.DefaultRobotCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.DriveWithJunctionRotation;
import org.firstinspires.ftc.teamcode.command.drive.SlowMode;
import org.firstinspires.ftc.teamcode.command.group.GrabAndLift;
import org.firstinspires.ftc.teamcode.command.group.LiftDown;
import org.firstinspires.ftc.teamcode.command.group.LiftUp;
import org.firstinspires.ftc.teamcode.command.lift.*;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
import org.firstinspires.ftc.teamcode.util.ConeStack;
import org.firstinspires.ftc.teamcode.util.Junction;
import org.firstinspires.ftc.teamcode.vision.JunctionDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.*;
import java.util.stream.Collectors;

@Config
@TeleOp(name = "Main TeleOp")
public final class MainOpMode extends BaseOpMode {

    private DefaultRobotCentricDrive robotCentricDrive;
    private DriveWithJunctionRotation driveWithJunctionRotation;
    private SlowMode slowMode;

    private MoveLiftPID moveLiftPID;

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

        driveWithJunctionRotation = new DriveWithJunctionRotation(drive, gamepadEx1::getLeftX,
                gamepadEx1::getRightX, gamepadEx1::getLeftY);

        robotCentricDrive = new DefaultRobotCentricDrive(drive, gamepadEx1::getLeftX,
                gamepadEx1::getRightX, gamepadEx1::getLeftY);

        slowMode = new SlowMode(drive, gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY);

        gb1(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(slowMode);

        gb1(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(driveWithJunctionRotation);

        gb2(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new GrabAndLift(lift, claw, goal), new ReleaseCone(claw));

        gb2(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .toggleWhenPressed(new SlideOut(slide), new SlideIn(slide));

        moveLiftPID = new MoveLiftPID(lift, gamepadEx2::getRightY);

        gb2(GamepadKeys.Button.Y)
                .whenPressed(new LiftUp(lift, slide, Junction.HIGH));

        gb2(GamepadKeys.Button.X)
                .whenPressed(new LiftUp(lift, slide, Junction.MEDIUM));

        gb2(GamepadKeys.Button.B)
                .whenPressed(new LiftUp(lift, slide, Junction.LOW));

        gb2(GamepadKeys.Button.A)
                .whenPressed(new LiftDown(lift, slide, claw));

        gb2(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SetConeStack(lift, ConeStack.FIRST));

        gb2(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetConeStack(lift, ConeStack.SECOND));

        gb2(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetConeStack(lift, ConeStack.THIRD));

        gb2(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetConeStack(lift, ConeStack.FOURTH));

        //forklift
        gb2(GamepadKeys.Button.RIGHT_BUMPER)
                .and(gb2(GamepadKeys.Button.A))
                        .whenActive(new SetJunction(lift, Junction.NONE));

        gb2(GamepadKeys.Button.RIGHT_BUMPER)
                .and(gb2(GamepadKeys.Button.B))
                        .whenActive(new SetJunction(lift, Junction.LOW));

        gb2(GamepadKeys.Button.RIGHT_BUMPER)
                .and(gb2(GamepadKeys.Button.X))
                        .whenActive(new SetJunction(lift, Junction.MEDIUM));

        gb2(GamepadKeys.Button.RIGHT_BUMPER)
                .and(gb2(GamepadKeys.Button.Y))
                        .whenActive(new SetJunction(lift, Junction.HIGH));


        register(drive, lift, claw, slide);
        drive.setDefaultCommand(robotCentricDrive);
        lift.setDefaultCommand(moveLiftPID);

    }

    @Override
    public void run() {
        super.run();
        tad("Target", lift.getTargetPosition());

        Point center = pipeline.uniqueCenter();
        if(center != null){
            double error = (center.x-640)/PIX_TO_DEGREE;
            drive.setHeading(imu.getHeading()-error);
        }
    }
}