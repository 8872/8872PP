package org.firstinspires.ftc.teamcode.command.claw;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;

public class ReleaseCone extends CommandBase {
    private final ClawSubsystem claw;
    public ReleaseCone(ClawSubsystem claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
