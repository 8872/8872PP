package org.firstinspires.ftc.teamcode.command.lift;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class SetJunctionPos extends CommandBase {
    private final LiftSubsystem arm;
    private final int junction;

    public SetJunctionPos(LiftSubsystem arm, int junction) {
        this.arm = arm;
        this.junction = junction;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setJunction(junction);
    }

    @Override
    public boolean isFinished() {
        Log.d("SetJunction status", "finished");
        return true;
    }
}
