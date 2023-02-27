package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.ClawSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public class CycleOneConeLeftHigh extends SequentialCommandGroup {
    public CycleOneConeLeftHigh(SampleMecanumDrive rrDrive, LiftSys lift, TurretSys turret, ArmSys arm, ClawSys claw, int coneHeight) {
        addCommands(
                //go to the given lift height while moving to the conestack
                new ParallelCommandGroup(
                        new DownSequenceWithPosition(lift, turret, arm, claw, coneHeight),
                        new FollowLeftConestackTrajectory(rrDrive)
                ),

                //grab and then raise up the arm
                new DelayedCommand(
                        claw.grab().alongWith(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 300)),
                        0),

                //move to high junction while doing the medium junction command group
                new DelayedCommand(
                        new ParallelCommandGroup(
                                new FollowLeftHighJunctionTrajectory(rrDrive),
                                new HighSequenceWithAngle(lift, turret, arm, 0.80845)
                        ),
                        0),

                //camera again
                new ParallelCommandGroup(
                        new AlignToPoleWithCamera(turret, 287),
                        new DelayedCommand(claw.release(), 500) // 650
                ), new WaitCommand(100)
        );

        addRequirements(lift, turret, claw, arm);
    }
}
