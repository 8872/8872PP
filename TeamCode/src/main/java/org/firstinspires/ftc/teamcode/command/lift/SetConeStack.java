package org.firstinspires.ftc.teamcode.command.lift;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class SetConeStack extends CommandBase {
    private final ArmSubsystem arm;
    private ArmSubsystem.ConeStack cone;
    public SetConeStack(ArmSubsystem arm, ArmSubsystem.ConeStack cone) {
        this.arm = arm;
        this.cone = cone;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setConeStack(cone);
    }

    @Override
    public boolean isFinished() {
        Log.d("SetConeStack status", "finished");
        return true;
    }
}
