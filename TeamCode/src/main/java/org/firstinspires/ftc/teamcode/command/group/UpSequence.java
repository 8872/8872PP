package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.Height;

public final class UpSequence extends SequentialCommandGroup {

    public UpSequence(LiftSubsystem lift, TurretSubsystem turret, ArmSubsystem arm, Height height, int turretDegrees) {
        addCommands(
                new ParallelCommandGroup(
                        lift.goTo(height),
                        arm.goTo(90)
                ), turret.goTo(turretDegrees),
                arm.goTo(45)
        );

        addRequirements(lift, turret, arm);
    }
}
