package org.firstinspires.ftc.teamcode.command.lift;

import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class MoveLift extends SimpleArmCommand {
    private final DoubleSupplier liftSpeed;
    public MoveLift(ArmSubsystem arm, DoubleSupplier liftSpeed) {
        super(arm);
        this.liftSpeed = liftSpeed;
    }

    @Override
    public void initialize() {
        arm.moveDr4b(liftSpeed.getAsDouble());
    }
}
