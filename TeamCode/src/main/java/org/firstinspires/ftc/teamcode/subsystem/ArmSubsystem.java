package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

@Config
public final class ArmSubsystem extends ProfiledServoSubsystem {
    public static double maxVelocity = 100;
    public static double maxAcceleration = 100;
    // arm down: 1
    // arm deposit: 0.6

    public static double down = 1;
    public static double deposit = 0.6;


    public ArmSubsystem(ServoEx turret) {
        super(turret, maxVelocity, maxAcceleration);
    }

    public Command down() {
        return goTo(down);
    }

    public Command deposit() {
        return goTo(deposit);
    }
}
