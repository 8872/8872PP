package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class SlideSubsystem extends SubsystemBase {
    private final ServoEx slide;
    public static int inPosition = 1;
    public static int outPosition = 0;

    public SlideSubsystem(ServoEx slide){
        this.slide = slide;
    }

    // moves slide to the in most position
    public void in(){
        slide.setPosition(inPosition);
    }

    // moves slide to the out most position
    public void out(){
        slide.setPosition(outPosition);
    }
}
