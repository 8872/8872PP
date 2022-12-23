package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class ManualLift extends CommandBase {
    private final LiftSubsystem arm;
    private final DoubleSupplier change;
    public ManualLift(LiftSubsystem arm, DoubleSupplier change){
        this.arm = arm;
        this.change = change;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.changeSetPoint(change.getAsDouble());
    }
}
