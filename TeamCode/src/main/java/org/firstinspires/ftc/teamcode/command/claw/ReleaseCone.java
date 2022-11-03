package org.firstinspires.ftc.teamcode.command.claw;

import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
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
