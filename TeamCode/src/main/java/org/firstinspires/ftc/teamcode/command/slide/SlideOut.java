package org.firstinspires.ftc.teamcode.command.slide;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;

public class SlideOut extends CommandBase {
    private final SlideSubsystem slide;
    public SlideOut(SlideSubsystem slide) {
        this.slide = slide;
        addRequirements(slide);
    }

    @Override
    public void initialize() {
        slide.out();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
