package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class SlideOut extends SimpleArmCommand{
    public SlideOut(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.slideOut();
    }
}
