package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class MoveLiftPID extends CommandBase {
    private final LiftSubsystem arm;
    private final DoubleSupplier supplier;

    public MoveLiftPID(LiftSubsystem arm, DoubleSupplier supplier) {
        this.arm = arm;
        this.supplier = supplier;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.resetEncoders();
        if(supplier.getAsDouble() != 0) {
            arm.changeSetPoint(supplier.getAsDouble());
            arm.loopPID();
        } else {
            arm.loopPID();
        }
    }
}
