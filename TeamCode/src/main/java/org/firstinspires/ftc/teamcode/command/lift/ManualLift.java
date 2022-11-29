package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ManualLift extends CommandBase {
    private ArmSubsystem arm;
    private DoubleSupplier change;
    public ManualLift(ArmSubsystem arm, DoubleSupplier change){
        this.arm = arm;
        this.change = change;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.changeSetpoint(change.getAsDouble());
    }
}
