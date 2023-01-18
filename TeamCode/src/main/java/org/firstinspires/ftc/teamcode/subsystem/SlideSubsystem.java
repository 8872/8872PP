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
    public static double inPosition = 0.55;
    public static double outPosition = 0.28;

    // TODO slide accels to slowly at the beginning. make asymmetric
    public static double maxVelocity = 1; //0.25
    public static double maxAcceleration = 1; //0.25

    private final TrapezoidProfile inProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
            new TrapezoidProfile.State(inPosition, 0),
            new TrapezoidProfile.State(0, 0));

    private final TrapezoidProfile outProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
            new TrapezoidProfile.State(outPosition, 0),
            new TrapezoidProfile.State(1, 0));

    private final ElapsedTime time = new ElapsedTime();
    public static double position;

    public SlideSubsystem(ServoEx slide){
        this.slide = slide;
    }

    // TODO finish tuning slide motion profile and make non blocking

    // moves slide to the in most position
    public void in(){
//        double initial = time.time();
//        double current;
//        TrapezoidProfile.State inState;
//        while (slide.getPosition() != inPosition) {
//            current = time.time();
//            inState = inProfile.calculate(current - initial);
//            slide.setPosition(inState.position);
//        }
        slide.setPosition(inPosition);
    }

    // moves slide to the out most position
    public void out(){
//        double initial = time.time();
//        double current;
//        TrapezoidProfile.State outState;
//        while (slide.getPosition() != outPosition) {
//            current = time.time();
//            outState = outProfile.calculate(current - initial);
//            slide.setPosition(outState.position);
//        }
        slide.setPosition(outPosition);
    }
    public void setPos(double pos){
        slide.setPosition(pos);
    }

    public double getPosition(){
        return slide.getPosition();
    }
}
