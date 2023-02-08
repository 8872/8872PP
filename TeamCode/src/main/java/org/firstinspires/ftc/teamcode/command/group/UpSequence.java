package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public final class UpSequence extends SequentialCommandGroup {

    public UpSequence(LiftSys lift, TurretSys turret, ArmSys arm, Height height, TurretSys.Pose pose) {
        addCommands(
                new ParallelCommandGroup(
                        lift.goTo(height),
                        arm.goTo(90)
                ), turret.goTo(pose),
                arm.goTo(45)
        );

        addRequirements(lift, turret, arm);
    }
}
