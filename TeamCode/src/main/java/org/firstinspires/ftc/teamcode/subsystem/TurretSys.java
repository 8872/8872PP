package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

@Config
public final class TurretSys extends ProfiledServoSubsystem {
    public static double maxVelocity = 100;
    public static double maxAcceleration = 100;
    // right forward: 0.88
    // left forward*: 0
    // right back: 0.57
    // left back: 0.29
    // start position: 0.435
    // left: 0.15
    // right: 0.715

    public enum Pose implements Position {
        RIGHT_FORWARD(0.88),
        LEFT_FORWARD(0),
        RIGHT_BACK(0.57),
        LEFT_BACK(0.29),
        LEFT(0.15),
        RIGHT(0.715),
        ZERO(0.435);

        private final double height;

        Pose(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    public TurretSys(ServoEx turret) {
        super(turret, maxVelocity, maxAcceleration);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                new TrapezoidProfile.State(Pose.ZERO.getHeight(), 0));
    }
}
