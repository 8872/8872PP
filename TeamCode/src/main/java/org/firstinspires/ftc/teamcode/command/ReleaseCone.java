package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class ReleaseCone extends SimpleArmCommand {
    public ReleaseCone(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.release();
    }
}
