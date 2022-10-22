package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class MoveConeGround extends SimpleArmCommand {
    public MoveConeGround(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.dr4bGround();
    }
}
