package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class MoveConeLow extends SimpleArmCommand {
    public MoveConeLow(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.dr4bLow();
    }
}
