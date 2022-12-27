package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class SlideSubsystem extends SubsystemBase {
    private final ServoEx slide;
    public static double inPosition = 1;
    public static double outPosition = 0;

    public SlideSubsystem(ServoEx slide){
        this.slide = slide;
    }

    // moves slide to the in most position
    public void in(){
        slide.setPosition(inPosition * (1.0/3));
        slide.setPosition(inPosition * (2.0/3));
        slide.setPosition(inPosition);
    }

    // moves slide to the out most position
    public void out(){
        slide.setPosition(outPosition + inPosition * (2.0/3));
        slide.setPosition(outPosition + inPosition * (1.0/3));
        slide.setPosition(outPosition);
    }
}
