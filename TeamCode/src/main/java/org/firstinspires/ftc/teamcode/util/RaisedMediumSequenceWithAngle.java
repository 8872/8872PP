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
public final class RaisedMediumSequenceWithAngle extends SequentialCommandGroup {
    //same as medium sequence but with a given turret set position
    public RaisedMediumSequenceWithAngle(LiftSys lift, TurretSys turret, ArmSys arm, double pos) {
        Log.d("ABC", "" + pos);
        addCommands(
                new ParallelCommandGroup(
                        lift.goTo(Height.MEDIUM.getHeight()-35),
                        arm.goTo(ArmSys.Pose.VERTICAL),
                        new DelayedCommand(turret.goTo(pos), 400),
                        new DelayedCommand(arm.goTo(ArmSys.Pose.HORIZONTAL, 2, 2),
                                600)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
