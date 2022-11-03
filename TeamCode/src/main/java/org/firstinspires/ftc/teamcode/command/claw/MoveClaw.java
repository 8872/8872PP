package org.firstinspires.ftc.teamcode.command.claw;

import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class MoveClaw extends SimpleArmCommand {
    private final DoubleSupplier clawSpeed;
    public MoveClaw(ArmSubsystem arm, DoubleSupplier clawSpeed) {
        super(arm);
        this.clawSpeed = clawSpeed;
    }

    @Override
    public void initialize() {
        arm.moveClaw(clawSpeed.getAsDouble());
    }
}
