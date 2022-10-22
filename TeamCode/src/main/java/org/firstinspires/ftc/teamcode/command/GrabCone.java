package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class GrabCone extends SimpleArmCommand {
    public GrabCone(ArmSubsystem arm) {
        super(arm);
    }

    @Override
    public void initialize() {
        arm.grab();
    }
}
