package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

@Config
public class ArmSubsystem extends ProfiledServoSubsystem {
    public static double maxVelocity = 100;
    public static double maxAcceleration = 100;

    public ArmSubsystem(ServoEx turret) {
        super(turret, maxVelocity, maxAcceleration);
    }
}
