package org.firstinspires.ftc.teamcode.command.slide;

import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class DelayedSlideOut extends SimpleArmCommand {
    public DelayedSlideOut(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.slideOut();
    }

    @Override
    public boolean isFinished() {
        return arm.slidePosReached();
    }
}

