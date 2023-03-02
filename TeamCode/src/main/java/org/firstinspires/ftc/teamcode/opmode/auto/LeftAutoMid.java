package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class LeftAutoMid extends BaseOpMode {

    //dr4b heights for conestack
    //firstCone is the dr4b height setpoint of the topmost cone
    public static int firstCone = -173;
    public static int secondCone = -148;
    public static int thirdCone = -129;
    public static int fourthCone = -103;
    public static int fifthCone = 0;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;


    // UNITS ARE METERS
    double tagsize = 0.166;

    private AprilTagDetection tagOfInterest = null;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;


    @Override
    public void initialize() {
        super.initialize();


        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        //setting up pipeline for camera
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });




//        while (!isStarted() && !isStopRequested()) {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if (tagFound) {
//                    telemetry.addData("Tag:", tagOfInterest.id);
//                    telemetry.update();
//                }
//            }

            camera.closeCameraDevice();

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 180, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
            AtomicBoolean tagFound = new AtomicBoolean(false);

            FunctionalCommand scanTag = new FunctionalCommand(
                    () -> {},
                    () -> {
                        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                        if (currentDetections.size() != 0) {
                            tagFound.set(false);

                            for (AprilTagDetection tag : currentDetections) {
                                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                                    tagOfInterest = tag;
                                    tagFound.set(true);
                                    break;
                                }
                            }
                        }
                    }, e -> {},
                    tagFound::get
            );



            Trajectory nullPark = rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(48, 0.2, Math.toRadians(180)))
                    .build();

            Trajectory park0 = rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(45, 24.52, Math.toRadians(90)))
                    .build();

            Trajectory park1 = rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(48, 0.2, Math.toRadians(180)))
                    .build();

            Trajectory park2 = rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(47, -24.52, Math.toRadians(180)))
                    .build();

            camera.setPipeline(pipeline);
            turret.setPipeline(pipeline);
            //set initial pose estimate
            rrDrive.setPoseEstimate(new Pose2d(-36, -62, Math.toRadians(90)));


            turretServo.setPosition(0.43);

            schedule(
                    new SequentialCommandGroup(
                            //grab and lift when the auto starts

                            new DelayedCommand(turret.goTo(0.43), 0),
                            //drive to the medium junction while doing mediumSequence
                            new ParallelCommandGroup(
                                    new DelayedCommand(claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 350)
                                            .alongWith(scanTag)), 0),
                                    new InstantCommand(() -> camera.setPipeline(pipeline)),

                                    new DelayedCommand(new FollowPreloadTrajectoryL(rrDrive), 500),
                                    //this command lets me set it to a specific angle instead of one of the setpositions
                                    new DelayedCommand(new MediumSequenceWithAngle(lift, turret, arm, 0.0422535), 1500)
                            ),

                            //give the camera half a second to align (isn't enough, should increase for more consistency)
                            new ParallelCommandGroup(
                                    new DelayedCommand(new AlignToPoleWithCamera(turret, 15), 100),
                                    //releases after 0.65 seconds, the command group continues after 1
                                    new DelayedCommand(claw.release(), 600)
                            ),
                            //cycle cones
                            new CycleOneConeL(rrDrive, lift, turret, arm, claw, firstCone),
                            new CycleOneConeL(rrDrive, lift, turret, arm, claw, secondCone),
                            new CycleOneConeL(rrDrive, lift, turret, arm, claw, thirdCone),
                            new CycleOneConeL(rrDrive, lift, turret, arm, claw, fourthCone),
                            new CycleOneConeL(rrDrive, lift, turret, arm, claw, fifthCone),

                            //reset the lift after everything finishes
                            new DownSequence(lift, turret, arm, claw),
                            new SelectCommand(new HashMap<Object, Command>() {{
                                put(null, new FollowTrajectoryCommand(rrDrive, rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(48, 0.2, Math.toRadians(180)))
                                        .build()));

                                put(0, new FollowTrajectoryCommand(rrDrive, rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(45, 24.52, Math.toRadians(90)))
                                        .build()));

                                put(1, new FollowTrajectoryCommand(rrDrive, rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(48, 0.2, Math.toRadians(180)))
                                        .build()));

                                put(2, new FollowTrajectoryCommand(rrDrive, rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(47, -24.52, Math.toRadians(180)))
                                        .build()));
                            }}, this::getTagId)

                    )
            );


            HashMap<Object, Command> test = new HashMap<Object, Command>() {{
               put(null, new FollowTrajectoryCommand(rrDrive, rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                       .lineToLinearHeading(new Pose2d(48, 0.2, Math.toRadians(180)))
                       .build()));

                put(0, new FollowTrajectoryCommand(rrDrive, rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(45, 24.52, Math.toRadians(90)))
                        .build()));

                put(1, new FollowTrajectoryCommand(rrDrive, rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(48, 0.2, Math.toRadians(180)))
                        .build()));

                put(2, new FollowTrajectoryCommand(rrDrive, rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(47, -24.52, Math.toRadians(180)))
                        .build()));
            }};

    }

    private Integer getTagId() {
        return tagOfInterest.id;
    }
}