package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.util.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RightAutoMid extends BaseOpMode {

    //dr4b heights for conestack
    //firstCone is the dr4b height setpoint of the topmost cone
    public static int firstCone = -172;
    public static int secondCone = -150;
    public static int thirdCone = -131;
    public static int fourthCone = -105;
    public static int fifthCone = 0;

    @Override
    public void initialize() {
        super.initialize();

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
        rrDrive.setPoseEstimate(new Pose2d(36, -62, Math.toRadians(90)));


        turretServo.setPosition(0.43);

        schedule(
                new SequentialCommandGroup(
                        //grab and lift when the auto starts

                        new DelayedCommand(turret.goTo(0.43), 0),
                        //drive to the medium junction while doing mediumSequence
                        new ParallelCommandGroup(
                                new DelayedCommand(claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 350)), 0),
                                new DelayedCommand(new FollowPreloadTrajectory(rrDrive), 500),
                                //this command lets me set it to a specific angle instead of one of the setpositions
                                new DelayedCommand(new RaisedMediumSequenceWithAngle(lift, turret, arm, 0.81127), 1500)
                        ),

                        //give the camera half a second to align (isn't enough, should increase for more consistency)
                        new ParallelCommandGroup(
                                new DelayedCommand(new AlignToPoleWithCamera(turret, 288),100),
                                //releases after 0.65 seconds, the command group continues after 1
                                new DelayedCommand(claw.release(), 1200)
                        ),
                        //cycle cones
                        new CycleOneCone(rrDrive, lift, turret, arm, claw, firstCone),
                        new NoAlignCycle(rrDrive, lift, turret, arm, claw, secondCone),
                        new NoAlignCycle(rrDrive, lift, turret, arm, claw, thirdCone),
                        new NoAlignCycle(rrDrive, lift, turret, arm, claw, fourthCone),
                        new NoAlignCycle(rrDrive, lift, turret, arm, claw, fifthCone),

                        //reset the lift after everything finishes
                        new DownSequence(lift, turret, arm, claw)

                )
        );
    }
}
