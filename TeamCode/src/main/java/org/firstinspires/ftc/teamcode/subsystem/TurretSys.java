package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

@Config
public final class TurretSys extends ProfiledServoSubsystem {
    public static double maxVelocity = 3;
    public static double maxAcceleration = 3;

    public enum Pose implements Position {
        RIGHT_FORWARD(0.9875),
        LEFT_FORWARD(0.135),
        RIGHT_BACK(0.67),
        LEFT_BACK(0.43),
        LEFT(0.29),
        RIGHT(0.832),
        ZERO(0.557),
        ONE_EIGHTY(0);

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
        currentTarget = Pose.ZERO.getHeight();
    }
}
