package org.firstinspires.ftc.teamcode.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class MoveLift extends CommandBase {
    private final DoubleSupplier liftSpeed;
    protected final ArmSubsystem arm;

    public MoveLift(ArmSubsystem arm, DoubleSupplier liftSpeed) {
        this.arm = arm;
        this.liftSpeed = liftSpeed;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.moveDr4b(liftSpeed.getAsDouble());
    }
}
