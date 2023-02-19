package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.NoRequirementInstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ProfiledServoSubsystem extends SubsystemBase {
    private final ServoEx turret;

    protected double maxVelocity = 3;
    protected double maxAcceleration = 3;

    protected TrapezoidProfile profile;

    protected double currentTarget;
    protected double previousTarget;

    private final ElapsedTime time = new ElapsedTime();
    private double initial;

    public ProfiledServoSubsystem(ServoEx turret, double maxVelocity, double maxAcceleration) {
        this.turret = turret;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        initial = time.time();
    }

    public ProfiledServoSubsystem(ServoEx turret) {
        this.turret = turret;
    }

    public Command goTo(double position, double velocity, double acceleration) {
        return new NoRequirementInstantCommand(() -> {
            initial = time.time();
            previousTarget = currentTarget;
            currentTarget = position;
            profile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(velocity, acceleration),
                    new TrapezoidProfile.State(currentTarget, 0),
                    new TrapezoidProfile.State(previousTarget, 0));
        })
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    public Command goTo(Position position, double velocity, double acceleration) {
        return goTo(position.getHeight(), velocity, acceleration);
    }

    public Command goTo(Position position) {
        return goTo(position.getHeight(), maxVelocity, maxAcceleration);
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
