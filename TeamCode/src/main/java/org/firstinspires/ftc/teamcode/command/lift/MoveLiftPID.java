package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class MoveLiftPID extends CommandBase {
    private final LiftSubsystem lift;
    private final DoubleSupplier supplier;

    public MoveLiftPID(LiftSubsystem lift, DoubleSupplier supplier) {
        this.lift = lift;
        this.supplier = supplier;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.checkLimitSwitch();
        if(supplier.getAsDouble() != 0) {
            lift.changeSetPoint(supplier.getAsDouble());
            lift.updatePID();
        } else {
            lift.updatePID();
        }
//        Log.d("asd", "loop");
    }
}
