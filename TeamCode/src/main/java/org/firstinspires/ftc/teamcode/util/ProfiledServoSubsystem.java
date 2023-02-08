package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ProfiledServoSubsystem extends SubsystemBase {
    private final ServoEx turret;

    protected double maxVelocity = 100;
    protected double maxAcceleration = 100;

    private TrapezoidProfile profile;

    private double currentTarget;

    private final ElapsedTime time = new ElapsedTime();
    private double initial;

    public ProfiledServoSubsystem(ServoEx turret, double maxVelocity, double maxAcceleration) {
        this.turret = turret;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public ProfiledServoSubsystem(ServoEx turret) {
        this.turret = turret;
    }

    public Command goTo(double position) {
        initial = time.time();
        return new InstantCommand(() -> {
            currentTarget = position;
            profile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                    new TrapezoidProfile.State(currentTarget, 0));
        }, this)
                .andThen(new WaitUntilCommand(this::atTarget));

    }

    public Command goTo(Position position) {
        return goTo(position.getHeight());
    }

    private boolean atTarget() {
        return turret.getPosition() == currentTarget;
    }

    @Override
    public void periodic() {
        double current = time.time();
        TrapezoidProfile.State outState = profile.calculate(current - initial);
        turret.setPosition(outState.position);
    }
}
