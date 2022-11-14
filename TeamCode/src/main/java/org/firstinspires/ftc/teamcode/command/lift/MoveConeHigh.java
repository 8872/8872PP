package org.firstinspires.ftc.teamcode.command.lift;

import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class MoveConeHigh extends SimpleArmCommand {
    public MoveConeHigh(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        System.out.println("high gggggg");
        arm.dr4bHigh();
    }
}
