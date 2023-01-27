package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class SlideSubsystem extends SubsystemBase {
    private final ServoEx slide;
    public static double inPosition = 0.55;
    public static double outPosition = 0.28;

    public SlideSubsystem(ServoEx slide){
        this.slide = slide;
    }

    public Command in(){
        return new InstantCommand(() -> slide.setPosition(inPosition), this);
    }

    public Command out(){
        return new InstantCommand(() -> slide.setPosition(outPosition), this);
    }

    public void setPos(double pos){
        slide.setPosition(pos);
    }

    public double getPosition(){
        return slide.getPosition();
    }
}
