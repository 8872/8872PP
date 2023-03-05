package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.command.NoRequirementInstantCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.LeftAutoMid;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.ClawSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public class NoAlignCycleL extends SequentialCommandGroup {

    public static double turretPosition = 350;

    public NoAlignCycleL(SampleMecanumDrive rrDrive, LiftSys lift, TurretSys turret, ArmSys arm, ClawSys claw, int coneHeight) {
        addCommands(
                //go to the given lift height while moving to the conestack
                new ParallelCommandGroup(
                        new DownSequenceWithPosition(lift, turret, arm, claw, coneHeight),
                        new FollowConestackTrajectoryL(rrDrive)
                ),
                //move to high junction while doing the medium junction command group
                new ParallelCommandGroup(
                        new DelayedCommand(claw.grab().alongWith(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 300)),100),
                        new DelayedCommand(
                                new ParallelCommandGroup(
                                        new FollowMidJunctionTrajectoryL(rrDrive),
                                        new InstantCommand(() -> LeftAutoMid.yDrift += 0.075),
                                        new InstantCommand(() -> LeftAutoMid.xDrift += 0.05),
                                        new DelayedCommand(new MediumSequenceWithAngleL(lift, turret, arm, (NoAlignCycle.turretPosition/355)),50)
                                ), 500
                        )
                        //new DelayedCommand(new InchDiagonally(rrDrive), 1600),
                ),
                new DelayedCommand(claw.release(),0),
                new WaitCommand(200)
        );

        addRequirements(lift, turret, claw, arm);
    }

    public static void setPos(double pos){
        Log.d("qwertyuio", ""+pos);
        turretPosition = pos;
    }
}
