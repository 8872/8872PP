package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.util.AngleController;

@Config
public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private final RevIMU imu;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
    private final AngleController controller = new AngleController(kP, kI, kD, 0);
    private double output;

    public static double slowFactor = 3.5;

    public DriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu){
        this.imu = imu;
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


    public void setHeading(double degrees){
        controller.setSetPoint(degrees);
    }

    public void updatePID(){
        output = controller.calculate(imu.getHeading());
        drive.driveWithMotorPowers(output, -output, output, -output);
    }

}
