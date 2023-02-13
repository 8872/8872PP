package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.util.ProfiledAngleController;

import java.util.function.DoubleSupplier;

@Config
public class DriveSys extends SubsystemBase {
    private final MecanumDrive drive;
    private final RevIMU imu;
    public static double kP = 0.06;
    public static double kI = 0;
    public static double kD = 0.0035;
    public static double maxVelocity = 20;
    public static double maxAcceleration = 20;
    private final ProfiledAngleController controller = new ProfiledAngleController(kP, kI, kD, 0,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    private double output;

    public static int joystickTransformFactor = 30;

    public static double slowFactor = 3.5;

    private double target;

    public DriveSys(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu) {
        this.imu = imu;
        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    public Command fieldCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                DoubleSupplier turnSpeed, DoubleSupplier gyroAngle) {
        return new RunCommand(
                () -> drive.driveFieldCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                        turnSpeed.getAsDouble(), gyroAngle.getAsDouble()),
                this
        );
    }

    public Command robotCentric(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> drive.driveRobotCentric(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                        turnSpeed.getAsDouble()),
                this
        );
    }

    public Command slowMode(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                            DoubleSupplier turnSpeed) {
        return new RunCommand(
                () -> drive.driveRobotCentric(strafeSpeed.getAsDouble() / slowFactor,
                        forwardSpeed.getAsDouble() / slowFactor,
                        turnSpeed.getAsDouble() / slowFactor),
                this
        );
    }

    public double getOutput() {
        return output;
    }

    public double getTarget() {
        return target;
    }

    // desmos: https://www.desmos.com/calculator/j2e6yaorld
    public double joystickTransform(double input) {
        return (1.0 / (joystickTransformFactor - 1))
                * Math.signum(input)
                * (Math.pow(joystickTransformFactor, Math.abs(input)) - 1);
    }
}
