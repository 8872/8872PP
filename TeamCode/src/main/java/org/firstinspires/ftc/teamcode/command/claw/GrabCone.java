package org.firstinspires.ftc.teamcode.command.claw;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;

public class GrabCone extends CommandBase {
    private final ClawSubsystem claw;
    public GrabCone(ClawSubsystem claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.grab();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
