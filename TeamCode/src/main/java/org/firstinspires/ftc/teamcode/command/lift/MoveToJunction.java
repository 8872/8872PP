package org.firstinspires.ftc.teamcode.command.lift;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class MoveToJunction extends CommandBase {
    private final LiftSubsystem lift;

    public MoveToJunction(LiftSubsystem lift) {
        this.lift = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.updatePID();
    }

    @Override
    public boolean isFinished() {
        return lift.atTarget();
    }
}
