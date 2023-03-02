package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;

public class SetTargetPos extends CommandBase {

    private double targetPos;

    public SetTargetPos(double targetPos) {
        Log.d("asd", "targetPos: " + targetPos);
        this.targetPos = targetPos;
    }
    @Override
    public void initialize(){
        //NoAlignCycle.turretPosition = targetPos;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
