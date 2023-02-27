package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.util.DelayedCommand;
import org.firstinspires.ftc.teamcode.util.FollowLeftPreloadTrajectory;
import org.firstinspires.ftc.teamcode.util.HighSequenceWithAngle;

@Autonomous
public class LeftTrajecTest extends BaseOpMode{
    @Override
    public void initialize() {
        super.initialize();

        rrDrive.setPoseEstimate(new Pose2d(-36, -62, Math.toRadians(90)));

        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                claw.grab().alongWith(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB),100)),
                                new DelayedCommand(new FollowLeftPreloadTrajectory(rrDrive), 150),
                                //this command lets me set it to a specific angle instead of one of the setpositions
                                new DelayedCommand(new HighSequenceWithAngle(lift, turret, arm, 0.78028), 150)
                        )

                )
        );
    }
    @Override
    public void run(){
        super.run();
        return;
    }
}
