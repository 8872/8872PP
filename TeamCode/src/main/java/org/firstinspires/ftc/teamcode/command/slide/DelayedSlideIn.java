package org.firstinspires.ftc.teamcode.command.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class DelayedSlideIn extends CommandBase {
    private final ElapsedTime timeElapsed = new ElapsedTime();
    private final ArmSubsystem arm;
    private final double waitTime;
    public DelayedSlideIn(ArmSubsystem arm, double waitTime) {
        this.waitTime = waitTime;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.slideIn();
        timeElapsed.reset();
    }

    @Override
    public boolean isFinished() {
        return timeElapsed.seconds() >= waitTime;
    }
}
