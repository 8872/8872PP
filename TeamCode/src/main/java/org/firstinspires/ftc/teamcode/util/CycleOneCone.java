package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.ClawSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public class CycleOneCone extends SequentialCommandGroup {
    public CycleOneCone(SampleMecanumDrive rrDrive, LiftSys lift, TurretSys turret, ArmSys arm, ClawSys claw, int coneHeight) {
        addCommands(
                //go to the given lift height while moving to the conestack
                new ParallelCommandGroup(
                        new DownSequenceWithPosition(lift, turret, arm, claw, coneHeight),
                        new FollowConestackTrajectory(rrDrive)
                ),

                //grab and then raise up the arm
                new DelayedCommand(
                        claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 200)),
                        0),

                //move to high junction while doing the medium junction command group
                new DelayedCommand(
                        new ParallelCommandGroup(
                                new FollowHighJunctionTrajectory(rrDrive),
                                new MediumSequenceWithAngle(lift, turret, arm, 0.82535)
                        ),
                        0),

                //camera again
                new ParallelCommandGroup(
                        new AlignToPoleWithCamera(turret),
                        new DelayedCommand(claw.release(), 650)
                )
        );

        addRequirements(lift, turret, claw, arm);
    }
}
