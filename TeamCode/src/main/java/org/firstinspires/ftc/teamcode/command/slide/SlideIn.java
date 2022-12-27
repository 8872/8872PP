package org.firstinspires.ftc.teamcode.command.slide;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;

public class SlideIn extends CommandBase {
    private final SlideSubsystem slide;
    public SlideIn(SlideSubsystem slide) {
        this.slide = slide;
        addRequirements(slide);
    }

    @Override
    public void initialize() {
        slide.in();
    }
}
