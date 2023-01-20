package org.firstinspires.ftc.teamcode.command.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HeadingPID extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier strafeSpeed,  forwardSpeed, turnSpeed;


    public HeadingPID(DriveSubsystem drive, DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                      DoubleSupplier turnSpeed) {
        this.drive = drive;
        this.strafeSpeed = strafeSpeed;
        this.forwardSpeed = forwardSpeed;
        this.turnSpeed = turnSpeed;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.updatePID();
        drive.driveRobotCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                    turnSpeed.getAsDouble());


    }
}