package org.firstinspires.ftc.teamcode.command.lift;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;

public class SetJunction extends CommandBase {
    private final LiftSubsystem lift;
    private final Junction junction;

    public SetJunction(LiftSubsystem lift, Junction junction) {
        this.lift = lift;
        this.junction = junction;
        addRequirements(lift);
    }

    @Override
    public void initialize() { lift.setJunction(junction); }

    @Override
    public boolean isFinished() {
        Log.d("SetJunction status", "finished");
        return true;
    }
}
