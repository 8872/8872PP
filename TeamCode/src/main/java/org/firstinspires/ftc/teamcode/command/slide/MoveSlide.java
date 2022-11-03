package org.firstinspires.ftc.teamcode.command.slide;

import org.firstinspires.ftc.teamcode.command.SimpleArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class MoveSlide extends SimpleArmCommand {
    private final DoubleSupplier slideSpeed;

    public MoveSlide(ArmSubsystem arm, DoubleSupplier slideSpeed) {
        super(arm);
        this.slideSpeed = slideSpeed;
    }

    @Override
    public void initialize() {
        arm.moveSlide(slideSpeed.getAsDouble());
    }
}
