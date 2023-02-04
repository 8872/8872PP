package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class ProfiledServoSubsystem extends SubsystemBase {
    private final ServoEx turret;

    protected double maxVelocity = 100;
    protected double maxAcceleration = 100;

    private TrapezoidProfile profile;

    private int currentTarget;

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

    public Command rotateToNow(int degrees) {
        return new InstantCommand(() -> turret.turnToAngle(degrees), this);
    }

    public Command rotateTo(int degrees) {
        initial = time.time();
        return new InstantCommand(() -> {
            currentTarget = degrees;
            profile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                    new TrapezoidProfile.State(currentTarget, 0));
        }, this)
                .andThen(new WaitUntilCommand(this::atTarget));

    }

    private boolean atTarget() {
        return turret.getPosition() == currentTarget;
    }

    @Override
    public void periodic() {
        double current = time.time();
        TrapezoidProfile.State outState = profile.calculate(current - initial);
        turret.turnToAngle(outState.position);
    }
}
