package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class SetTick extends CommandBase {
    private final LiftSubsystem lift;
    private final int goal;

    public SetTick(LiftSubsystem lift, int goal){
        this.lift = lift;
        this.goal = goal;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.setJunction(goal);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
