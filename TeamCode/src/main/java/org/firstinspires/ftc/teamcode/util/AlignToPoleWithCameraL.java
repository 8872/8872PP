package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public class AlignToPoleWithCameraL extends CommandBase {
    TurretSys turret;
    ElapsedTime time;
    double setPosition;
    double delay = -1;

    public AlignToPoleWithCameraL(TurretSys turret, double setPosition) {
        this.turret = turret;
        this.setPosition = setPosition;
        time = new ElapsedTime();
    }

    public AlignToPoleWithCameraL(TurretSys turret, double delay, double setPosition){
        this.turret = turret;
        this.setPosition = setPosition;
        time = new ElapsedTime();
        this.delay = delay;
    }

    @Override
    public void initialize() {
        //start tracking and reset the timer
        turret.setSetPosition(setPosition);
        turret.setLimited(true);
        turret.startTracking();
        time.reset();
    }

    @Override
    public boolean isFinished() {
        //finish after 1 second
        //the camera automatically stops tracking when goTo for turret is called
        if(delay != -1){
            return time.seconds()>=delay;
        }
        Log.d("test", ""+turret.getTarget());
        if(time.seconds() >= 1.5){
            Log.d("testSet", "+"+turret.getTarget());
            NoAlignCycle.setPos(turret.getTarget()-5);
        }
        return time.seconds() >= 1.6;
    }
}
