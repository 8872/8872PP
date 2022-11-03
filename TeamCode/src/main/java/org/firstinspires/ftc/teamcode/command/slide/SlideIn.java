package org.firstinspires.ftc.teamcode.command.slide;

import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class SlideIn extends SimpleArmCommand {
    public SlideIn(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.slideIn();
    }
}
