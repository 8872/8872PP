package org.firstinspires.ftc.teamcode.command.lift;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.ConeStack;

public class SetConeStack extends CommandBase {
    private final LiftSubsystem lift;
    private final ConeStack cone;

    public SetConeStack(LiftSubsystem lift, ConeStack cone) {
        this.lift = lift;
        this.cone = cone;
        addRequirements(lift);
    }

    @Override
    public void initialize() {
        lift.setConeStack(cone);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
