package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class SimpleArmCommand extends CommandBase {
    protected final ArmSubsystem arm;

    public SimpleArmCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
