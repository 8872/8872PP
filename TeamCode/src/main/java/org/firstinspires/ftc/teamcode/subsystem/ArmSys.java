package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

@Config
public final class ArmSys extends ProfiledServoSubsystem {
    public static double maxVelocity = 2;
    public static double maxAcceleration = 2;
    // arm down: 1
    // arm deposit: 0.6

    public enum Pose implements Position {
        DOWN(1),
        DEPOSIT(0.6),
        GRAB(0.82),
        FORKLIFT(0.85);

        private final double height;

        Pose(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    public ArmSys(ServoEx turret) {
        super(turret, maxVelocity, maxAcceleration);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                new TrapezoidProfile.State(Pose.DOWN.getHeight(), 0));
        currentTarget = Pose.DOWN.getHeight();
    }
}
