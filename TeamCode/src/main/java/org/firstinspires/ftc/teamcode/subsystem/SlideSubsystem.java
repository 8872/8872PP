package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class SlideSubsystem extends SubsystemBase {
    private final ServoEx slide;
    public static double inPosition = 1;
    public static double outPosition = 0;

    public static double maxVelocity = 0.25;
    public static double maxAcceleration = 0.25;
    private final TrapezoidProfile inProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
            new TrapezoidProfile.State(1, 0),
            new TrapezoidProfile.State(0, 0));
    private final TrapezoidProfile outProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
            new TrapezoidProfile.State(0, 0),
            new TrapezoidProfile.State(1, 0));

    private final ElapsedTime time = new ElapsedTime();
    public static double position;

    public SlideSubsystem(ServoEx slide){
        this.slide = slide;
    }

    // TODO finish tuning slide motion profile and make non blocking

    // moves slide to the in most position
    public void in(){
        double initial = time.time();
        double current;
        TrapezoidProfile.State inState;
        while (slide.getPosition() != 1) {
            current = time.time();
            inState = inProfile.calculate(current - initial);
            Log.d("slide position", ""+inState.position);
            Log.d("elapsed time", ""+(current - initial));
            slide.setPosition(inState.position);
        }
    }

    // moves slide to the out most position
    public void out(){

        slide.setPosition(outPosition);
    }
}
