package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class MoveConeHigh extends SimpleArmCommand {
    public MoveConeHigh(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.dr4bHigh();
    }
}
