package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.command.group.GrabAndLift;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DelayedClaw extends CommandBase {
    public final LiftSubsystem lift;
    public final ClawSubsystem claw;
    public final DoubleSupplier delayTime;
    public final BooleanSupplier grabAction;
    private ElapsedTime timer = new ElapsedTime();
    public DelayedClaw(LiftSubsystem lift, ClawSubsystem claw, DoubleSupplier delayTime, BooleanSupplier grabAction){
        this.lift = lift;
        this.claw = claw;
        this.delayTime = delayTime;
        this.grabAction = grabAction;
        addRequirements(claw);
    }
    @Override
    public void initialize() {
        timer.reset();
        if(grabAction.getAsBoolean()){
            claw.grab();
        }else{
            new ScheduleCommand(new GrabAndLift(lift, claw, lift.getLeftEncoderValue()-100));
        }
    }

    @Override
    public boolean isFinished() {
        return timer.seconds()>delayTime.getAsDouble();
    }
}
