package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

@Config
public final class MediumSequenceWithAngleL extends SequentialCommandGroup {
    //same as medium sequence but with a given turret set position
    public MediumSequenceWithAngleL(LiftSys lift, TurretSys turret, ArmSys arm, double pos) {
        Log.d("qwerty", ""+pos);
        addCommands(
                new ParallelCommandGroup(
                        lift.goTo(Height.MEDIUM.getHeight()+5),
                        arm.goTo(ArmSys.Pose.VERTICAL),
                        new DelayedCommand(turret.goTo(false), 350),
                        new DelayedCommand(arm.goTo(0.9, 0.7, 0.7),
                                600)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
