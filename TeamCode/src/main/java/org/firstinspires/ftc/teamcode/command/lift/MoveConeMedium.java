package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class MoveConeMedium extends SimpleArmCommand {
    public MoveConeMedium(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.dr4bMedium();
    }
}
