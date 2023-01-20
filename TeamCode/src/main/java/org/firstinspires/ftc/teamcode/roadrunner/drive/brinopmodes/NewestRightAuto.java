package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubAuto;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


//TODO Improvements: make slide not jerk as much with delayed full extension and make the conestack timer increase linearly instead of being constant
//TODO make the slide extend mostly out in preload to save half a second
@Config
@Autonomous(name = "newer right side")
public class NewestRightAuto extends LinearOpMode {

    int reverse = -1;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    // UNITS ARE METERS
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;

    public double initial_x_pos = 54.5;//55.56;
    public double initial_y_pos = -0.7;//-2;
    public double initial_turn_angle = 123;
    public double spline_x_pos = 49;//51;
    public double spline_y_pos = 7.16;//5.5;
    public double retrieve_y_pos = 26.5;//27.33;
    public double deposit_x_pos = 54.8;//54.75;

    public double deposit_y_pos = -0.7;//-3.33;
    public double x_change = 0.75;
    public double y_change = 0.1;

    private MotorEx dr4bLeftMotor, dr4bRightMotor;
    private SimpleServo claw, slide;
    private TouchSensor limitSwitch;

    private SampleMecanumDrive drive;
    private LiftSubAuto liftSub;
    private ClawSubsystem clawSub;
    private SlideSubsystem slideSub;

    int pickupPosition = -115;
    int coneCounter = 5;

    private enum DRIVE_PHASE {
        WAIT_FOR_PRELOAD,
        SLIDE,
        PRELOAD,
        DEPOSIT,
        WAIT_FOR_DEPOSIT,
        MOVE_TO_RETRIEVE,
        WAIT,
        RETRIEVE,
        WAIT_FOR_GRAB,
        BRUH,
        DEPOSIT_SLIDE,
        PARK,
        IDLE
    }
    boolean delayedExtend = false;
    boolean delayedLift = false;
    double waitTime = -0.05;
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime delayTimer = new ElapsedTime();
    ElapsedTime wait100 = new ElapsedTime();
    ElapsedTime wait250 = new ElapsedTime();
    ElapsedTime wait750 = new ElapsedTime();

    DRIVE_PHASE currentState = DRIVE_PHASE.IDLE;
    Pose2d startPose = new Pose2d(-1.5, 0, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        double storedSplineX = spline_x_pos;
        double storedDepositX = deposit_x_pos;
        double storedDepositY = deposit_y_pos;

        dr4bLeftMotor = new MotorEx(hardwareMap, "dr4bLeft");
        dr4bRightMotor = new MotorEx(hardwareMap, "dr4bRight");
        dr4bLeftMotor.resetEncoder();
        dr4bRightMotor.resetEncoder();

        claw = new SimpleServo(hardwareMap, "claw", 0, 120);
        slide = new SimpleServo(hardwareMap, "slide", 0, 120);

        limitSwitch = hardwareMap.get(TouchSensor.class, "touch");

        drive = new SampleMecanumDrive(hardwareMap);
        slideSub = new SlideSubsystem(slide);
        liftSub = new LiftSubAuto(dr4bLeftMotor,dr4bRightMotor,limitSwitch);
        clawSub = new ClawSubsystem(claw);
        initCamera();

        drive.setPoseEstimate(startPose);

        clawSub.grab();
        liftSub.setJunction(-50);
        delayTimer.reset();
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if(tagFound)
                {
                    telemetry.addData("Tag:", tagOfInterest.id);
                    telemetry.addData("initialized", true);
                }
            }
            liftSub.updatePID();
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;


        currentState = DRIVE_PHASE.WAIT_FOR_PRELOAD;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case WAIT_FOR_PRELOAD:
                    liftSub.setJunction(Junction.LOW);
                    liftTimer.reset();
                    delayedLift = true;
                    drive.followTrajectoryAsync(drive.trajectoryBuilder(startPose)
                            .lineToLinearHeading(new Pose2d(initial_x_pos-1,(initial_y_pos+1.5)*reverse, Math.toRadians(initial_turn_angle)*reverse)//, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            )//SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .build());
                    slideSub.setPos(0.42);
                    currentState = DRIVE_PHASE.SLIDE;
                    break;
                case SLIDE:
                    if(!drive.isBusy()){
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(initial_x_pos,initial_y_pos*reverse, Math.toRadians(initial_turn_angle)*reverse)//, SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                )//SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                        slideSub.out();
                        wait250.reset();
                        currentState = DRIVE_PHASE.DEPOSIT;
                    }
                    break;
                case DEPOSIT_SLIDE:
                    if (!drive.isBusy()) {
                        slideSub.out();
                        wait750.reset();
                        currentState = DRIVE_PHASE.DEPOSIT;
                    }
                    break;
                case DEPOSIT:
                    if (!drive.isBusy() && wait750.seconds()>=0.4 && wait250.seconds() >= 0.7) {
                        currentState = DRIVE_PHASE.WAIT_FOR_DEPOSIT;
                        wait250.reset();
                    }
                    break;

                case WAIT_FOR_DEPOSIT:
                    if (wait250.seconds() >= 0) {
                        clawSub.release();
                        wait100.reset();
                        currentState = DRIVE_PHASE.MOVE_TO_RETRIEVE;
                    }
                    break;
                case MOVE_TO_RETRIEVE:
                    if(wait100.seconds() >= 0.1){
                        slideSub.in();
                        liftSub.setJunction(pickupPosition);
                        pickupPosition+=33;
                        if(coneCounter <= 0){
                            currentState = DRIVE_PHASE.PARK;
                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .turn(Math.toRadians(-45)*reverse)
                                    .build());
                        }else{
                            currentState = DRIVE_PHASE.WAIT;
                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .forward(2)
                                    .splineTo(new Vector2d(spline_x_pos, (spline_y_pos+10)*reverse), Math.toRadians(90)*reverse)
                                    .forward(Math.abs(retrieve_y_pos-spline_y_pos-10.5))
                                    .build());
                            spline_x_pos += x_change*reverse;
                            retrieve_y_pos -= y_change*reverse;
                        }
                        coneCounter--;

                    }
                    break;

                case WAIT:
                    if(!drive.isBusy()){
                        liftTimer.reset();
                        currentState=DRIVE_PHASE.RETRIEVE;
                    }
                    break;
                case RETRIEVE:
                    if (liftTimer.seconds()>=waitTime) {
                        clawSub.grab();
                        wait250.reset();
                        currentState = DRIVE_PHASE.BRUH;
                    }
                    break;
                case BRUH:
                    if(wait250.seconds() >= 0.25){
                        liftSub.setJunction(Junction.MEDIUM);
                        liftTimer.reset();
                        delayedLift = true;
                        wait750.reset();
                        currentState = DRIVE_PHASE.WAIT_FOR_GRAB;
                        waitTime+=0.1;
                    }
                    break;
                case WAIT_FOR_GRAB:
                    if(wait750.seconds()>=0.5){
                        delayedExtend = true;
                        delayTimer.reset();
                        currentState = DRIVE_PHASE.DEPOSIT_SLIDE;
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .back(Math.abs(spline_y_pos-deposit_y_pos-10))
                                .splineTo(new Vector2d(deposit_x_pos, (deposit_y_pos*reverse)), Math.toRadians(303)*reverse)
                                .build());
                        deposit_x_pos += x_change*reverse;
                        deposit_y_pos -= y_change*reverse;
                    }
                    break;
                case PARK:
                    if(!drive.isBusy()){
                        if(tagOfInterest == null){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(48, 0.2*reverse, Math.toRadians(180)*reverse))
                                    .build());
                        }else if(tagOfInterest.id == 0) {
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(45, 24.52*reverse, Math.toRadians(90)*reverse))
                                    .build());
                        }else if(tagOfInterest.id == 1){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(48, 0.2*reverse, Math.toRadians(180)*reverse))
                                    .build());
                        }else if(tagOfInterest.id == 2){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(45, -24.52*reverse, Math.toRadians(90)*reverse))
                                    .build());
                        }
                        currentState = DRIVE_PHASE.IDLE;
                    }
                    break;
                case IDLE:
                    deposit_y_pos = storedDepositY;
                    deposit_x_pos = storedDepositX;
                    spline_x_pos = storedSplineX;
                    break;
            }
            if(delayedExtend && delayTimer.seconds()>=0.2){
                delayedExtend = false;
                slideSub.setPos(0.42);
            }

            if(delayedLift && liftTimer.seconds()>=0.75){
                liftSub.setJunction(Junction.HIGH);
                delayedLift = false;
            }

            drive.update();
            liftSub.updatePID();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("drive phase", currentState);
            telemetry.update();
        }
    }
    public void initCamera(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280 , 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
}