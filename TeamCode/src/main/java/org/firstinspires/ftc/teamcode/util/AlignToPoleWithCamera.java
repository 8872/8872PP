package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public class AlignToPoleWithCamera extends CommandBase {
    TurretSys turret;
    ElapsedTime time;

    public AlignToPoleWithCamera(TurretSys turret) {
        this.turret = turret;
        time = new ElapsedTime();
    }

    @Override
    public void initialize() {
        //start tracking and reset the timer
        turret.startTracking();
        time.reset();
    }

    @Override
    public boolean isFinished() {
        //finish after 1 second
        //the camera automatically stops tracking when goTo for turret is called
        return time.seconds() >= 1;
    }
}
