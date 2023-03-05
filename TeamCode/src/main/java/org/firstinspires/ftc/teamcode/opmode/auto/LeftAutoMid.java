package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class LeftAutoMid extends BaseOpMode {

    ElapsedTime timer = new ElapsedTime();
    boolean started = false;
    AprilTagDetectionPipeline aprilTagPipeline;
    private AprilTagDetection tagOfInterest = null;
    private OpenCvWebcam camera2;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    public static double yDrift = 0;
    public static double xDrift = 0;

    boolean finished = false;
    boolean followPark = false;


    //dr4b heights for conestack
    //firstCone is the dr4b height setpoint of the topmost cone
    public static int firstCone = -163;
    public static int secondCone = -142;
    public static int thirdCone = -119;
    public static int fourthCone = -98;
    public static int fifthCone = 0;

    @Override
    public void initialize() {
        super.initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        aprilTagPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera2.setPipeline(aprilTagPipeline);
        camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera2.getFocusControl().setMode(FocusControl.Mode.Fixed);
                camera2.getExposureControl().setMode(ExposureControl.Mode.Manual);
                camera2.startStreaming(1280 , 720, OpenCvCameraRotation.UPRIGHT);

            }
            @Override
            public void onError(int errorCode)
            {

            }
        });

        //setting up pipeline for camera
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

        //set initial pose estimate
        rrDrive.setPoseEstimate(new Pose2d(-36, -62, Math.toRadians(90)));


        turretServo.setPosition(0.43);

        schedule(
                new SequentialCommandGroup(
                        //grab and lift when the auto starts

                        new DelayedCommand(turret.goTo(0.43), 0),
                        //drive to the medium junction while doing mediumSequence
                        new ParallelCommandGroup(
                                new DelayedCommand(claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 350)), 0),
                                new DelayedCommand(new FollowPreloadTrajectoryL(rrDrive), 800),
                                //this command lets me set it to a specific angle instead of one of the setpositions
                                new DelayedCommand(new RaisedMediumSequenceWithAngle(lift, turret, arm, 0.0422535), 1800)
                        ),

                        //give the camera half a second to align (isn't enough, should increase for more consistency)
                        new ParallelCommandGroup(
                                new DelayedCommand(new AlignToPoleWithCamera(turret, 15),100),
                                //releases after 0.65 seconds, the command group continues after 1
                                new DelayedCommand(claw.release(), 1200)
                        ),
                        //cycle cones
                        new CycleOneConeL(rrDrive, lift, turret, arm, claw, firstCone),
                        new NoAlignCycleL(rrDrive, lift, turret, arm, claw, secondCone),
                        new NoAlignCycleL(rrDrive, lift, turret, arm, claw, thirdCone),
                        new NoAlignCycleL(rrDrive, lift, turret, arm, claw, fourthCone),
                        new NoAlignCycleL(rrDrive, lift, turret, arm, claw, fifthCone),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> finished = true),

                                //reset the lift aft    er everything finishes
                                new DownSequenceWithPosition(lift, turret, arm, claw,0)
                        )
                )
        );
    }
    @Override
    public void run(){
        super.run();

        if(!started){
            timer.reset();
            started = true;
        }
        if(timer.seconds() < 4){
            ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();

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
                    Log.d("asd", ""+tagOfInterest.id);
                    telemetry.addData("Tag:", tagOfInterest.id);
                }
            }
            telemetry.update();
        }
        else {
            camera2.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                @Override
                public void onClose() {

                }
            });
        }

        if(finished){
            Log.d("finished", "finished");
            finished = false;
            followPark = true;
            if(tagOfInterest == null){
                rrDrive.followTrajectoryAsync(rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-59,-10,Math.toRadians(180)))
                        .build());
            }
            else{
                switch(tagOfInterest.id){
                    case 1:
                        rrDrive.followTrajectoryAsync(rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-57,-10, Math.toRadians(180)))
                                .build());
                        break;
                    case 2:
                        rrDrive.followTrajectoryAsync(rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-35,-10, Math.toRadians(180)))
                                .build());
                        break;
                    case 3:
                        rrDrive.followTrajectoryAsync(rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(-11,-10, Math.toRadians(180)))
                                .build());
                        break;
                }
            }
        }
        if(followPark){
            rrDrive.update();
        }
    }
}
