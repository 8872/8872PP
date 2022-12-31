package org.firstinspires.ftc.teamcode.command.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class SetHeading extends CommandBase {
    private final DriveSubsystem drive;
    private final double angle;

    public SetHeading(DriveSubsystem drive, double angle) {
        this.drive = drive;
        this.angle = angle;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setHeading(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
