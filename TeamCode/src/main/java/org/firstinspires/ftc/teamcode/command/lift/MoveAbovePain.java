package org.firstinspires.ftc.teamcode.command.lift;

import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class MoveAbovePain extends CommandBase {
    private final ArmSubsystem arm;

    public MoveAbovePain(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        if(arm.getLeftEncoderValue() > -550){
            arm.setJunction(-600);
        }
    }
    @Override
    public void execute() {
        arm.loopPID();
        arm.resetEncoders();
    }
    @Override
    public boolean isFinished() {
        return arm.getLeftEncoderValue()<-550;
    }
}
