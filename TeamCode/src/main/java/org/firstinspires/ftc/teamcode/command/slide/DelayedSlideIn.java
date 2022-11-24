package org.firstinspires.ftc.teamcode.command.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class DelayedSlideIn extends SimpleArmCommand {
    public DelayedSlideIn(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.slideIn();
    }

    @Override
    public boolean isFinished() {
        return arm.slidePosReached();
    }
}
