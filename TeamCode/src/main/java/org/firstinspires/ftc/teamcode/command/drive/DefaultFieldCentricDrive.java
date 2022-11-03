package org.firstinspires.ftc.teamcode.command.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultFieldCentricDrive extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier strafeSpeed,  forwardSpeed, turnSpeed, gyroAngle;

    public DefaultFieldCentricDrive(DriveSubsystem drive, DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                    DoubleSupplier turnSpeed, DoubleSupplier gyroAngle){
        this.drive = drive;
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.turnSpeed = turnSpeed;
        this.gyroAngle = gyroAngle;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble(), gyroAngle.getAsDouble());
    }
}
