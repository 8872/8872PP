package org.firstinspires.ftc.teamcode.command;

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
