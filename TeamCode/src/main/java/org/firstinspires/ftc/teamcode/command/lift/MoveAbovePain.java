package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class MoveAbovePain extends CommandBase {
    private final LiftSubsystem arm;

    public MoveAbovePain(LiftSubsystem arm) {
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
