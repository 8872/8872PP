package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

public final class ArmSys extends ProfiledServoSubsystem {
    public static double maxVelocity = 100;
    public static double maxAcceleration = 100;
    // arm down: 1
    // arm deposit: 0.6

    public enum Pose implements Position {
        DOWN(1),
        DEPOSIT(0.6),
        GRAB(0.9);

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
    }
}
