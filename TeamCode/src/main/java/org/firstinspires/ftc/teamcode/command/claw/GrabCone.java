package org.firstinspires.ftc.teamcode.command.claw;

import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class GrabCone extends SimpleArmCommand {
    public GrabCone(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.grab();
    }
}
