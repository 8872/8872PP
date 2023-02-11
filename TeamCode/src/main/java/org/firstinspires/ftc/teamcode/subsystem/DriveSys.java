package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.util.ProfiledAngleController;
import org.firstinspires.ftc.teamcode.util.ScuffedMecanumDrive;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

@Config
public class DriveSys extends SubsystemBase {
    private final ScuffedMecanumDrive drive;
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
        drive = new ScuffedMecanumDrive(fL, fR, bL, bR);
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

    public Command driveWithConeRotation(DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed,
                                         DoubleSupplier turnSpeed) {
        double[] motorPowers = drive.getMotorPowers(strafeSpeed.getAsDouble(), forwardSpeed.getAsDouble(),
                turnSpeed.getAsDouble());
        output = controller.calculate(imu.getHeading());
        double[] finalPowers;

        for (int i = 0; i < motorPowers.length; i++)
            motorPowers[i] -= output;

        double max = Math.abs(Arrays.stream(motorPowers).max().getAsDouble());
        if (max > 1) {
            for (int i = 0; i < motorPowers.length; i++)
                motorPowers[i] /= max;
        }
        finalPowers = motorPowers.clone();

        return new RunCommand(
                () -> drive.driveWithMotorPowers(finalPowers[0], finalPowers[1], finalPowers[2], finalPowers[3]),
                this
        );
    }


    public void setHeading(double degrees) {
        controller.setGoal(degrees);
        target = degrees;
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

    public boolean atDesiredAngle() {
        return Math.abs(target - imu.getHeading()) < 2;
    }

    @Override
    public void periodic() {
//        output = controller.calculate(imu.getHeading());
//        drive.driveWithMotorPowers(-output, -output, -output, -output);
    }
}
