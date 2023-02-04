package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

@Config
public final class TurretSubsystem extends ProfiledServoSubsystem {
    public static double maxVelocity = 100;
    public static double maxAcceleration = 100;
    // right forward: 0.93
    // left forward*: 0.07
    // right back: 0.66
    // left back: 0.35
    // start position: 0.51
    // left: 0.23
    // right:0.805

    public enum Position {
        RIGHT_FORWARD(0.93),
        LEFT_FORWARD(0.07),
        RIGHT_BACK(0.66),
        LEFT_BACK(0.35),
        LEFT(0.23),
        RIGHT(0.805),
        ZERO(0.51);

        private final double height;
        Position(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }


    public TurretSubsystem(ServoEx turret) {
        super(turret, maxVelocity, maxAcceleration);
    }
}
