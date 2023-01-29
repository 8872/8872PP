package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

@Config
public final class TurretSubsystem extends ProfiledServoSubsystem {
    public static double maxVelocity = 100;
    public static double maxAcceleration = 100;

    public TurretSubsystem(ServoEx turret) {
        super(turret, maxVelocity, maxAcceleration);
    }
}
