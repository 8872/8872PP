package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.ClawSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public class CycleOneConeL extends SequentialCommandGroup {
    public CycleOneConeL(SampleMecanumDrive rrDrive, LiftSys lift, TurretSys turret, ArmSys arm, ClawSys claw, int coneHeight) {
        addCommands(
                //go to the given lift height while moving to the conestack
                new ParallelCommandGroup(
                        new DownSequenceWithPosition(lift, turret, arm, claw, coneHeight),
                        new FollowConestackTrajectoryL(rrDrive)
                ),

                //grab and then raise up the arm
//                new DelayedCommand(
//                        claw.grab().alongWith(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 300)),
//                        0),

                //move to high junction while doing the medium junction command group
                new ParallelCommandGroup(
                        claw.grab().alongWith(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 300)),
                        new DelayedCommand(
                                new ParallelCommandGroup(
                                        new FollowMidJunctionTrajectoryL(rrDrive),
                                        new RaisedMediumSequenceWithAngle(lift, turret, arm, 0.0140845)
                                ), 500
                        )
                ),


                //camera again
                new ParallelCommandGroup(
                        new AlignToPoleWithCamera(turret, 5),
                        //new DelayedCommand(new SetTargetPos(turret.targetPos),1500),
                        new DelayedCommand(claw.release(), 1250) // 650
                ), new WaitCommand(100)
        );

        addRequirements(lift, turret, claw, arm);
    }
}