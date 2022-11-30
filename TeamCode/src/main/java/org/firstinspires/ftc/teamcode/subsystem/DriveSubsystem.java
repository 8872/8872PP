package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

@Config
public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    public static double slowFactor = 3.5;

    public DriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR){
        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed){
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle){
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle);
    }

    public void driveRobotCentricSlowMode(double strafeSpeed, double forwardSpeed, double turnSpeed){
        drive.driveRobotCentric(strafeSpeed / slowFactor, forwardSpeed / slowFactor, turnSpeed / slowFactor);
    }
}
