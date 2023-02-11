package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.util.ProfiledPIDFController;

@Config
public class LiftSys extends SubsystemBase {

    private final MotorEx left, right;
    private final TouchSensor limitSwitch;

    private final double TICKS_IN_DEGREES = 8192.0 / 360;

    public static double kP = 0.0019; //0.0016
    public static double kI = 0.056; //0.06
    public static double kD = 0.000175; //0.00018
    public static double kF = 0.085; //0.06
    public static double maxVelocity = 5000;
    public static double maxAcceleration = 5000;
    private final ProfiledPIDFController leftPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    private final ProfiledPIDFController rightPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    public static double tolerance = 10;

    public static int currentHeight = 0;

    public static int manualLiftSpeed = 30;

    public static int threshold = 20;

    public LiftSys(MotorEx left, MotorEx right, TouchSensor limitSwitch) {
        this.limitSwitch = limitSwitch;
        this.left = left;
        this.right = right;
        leftPIDF.setTolerance(tolerance);
        leftPIDF.setGoal(0);
        rightPIDF.setGoal(0);
    }

    private void setHeight(Height height) {
        currentHeight = height.getHeight();
        leftPIDF.setGoal(height.getHeight());
        rightPIDF.setGoal(height.getHeight());
    }

    private void setHeight(int tick) {
        currentHeight = tick;
        leftPIDF.setGoal(tick);
        rightPIDF.setGoal(tick);
    }

    public int getLeftEncoderValue() {
        return left.getCurrentPosition();
    }

    public boolean atTarget() {
        return left.getCurrentPosition() < currentHeight + threshold &&
                left.getCurrentPosition() > currentHeight - threshold;
    }


    public void changeSetPoint(double joystickInput) {
        leftPIDF.setGoal((int) (left.getCurrentPosition() + joystickInput * manualLiftSpeed));
        rightPIDF.setGoal((int) (right.getCurrentPosition() + joystickInput * manualLiftSpeed));
    }

    public int getCurrentGoal() {
        return currentHeight;
    }

    public boolean isGrabAndLiftIncompatible() {
        return currentHeight != Height.NONE.getHeight();
    }

    public Command goTo(Height junction) {
        return new InstantCommand(() -> setHeight(junction))
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    public Command goTo(int tick) {
        return new InstantCommand(() -> setHeight(tick))
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    @Override
    public void periodic() {
        if (limitSwitch.isPressed()) {
            left.resetEncoder();
            right.resetEncoder();
        }

        double output_left = leftPIDF.calculate(left.getCurrentPosition());
        double output_right = rightPIDF.calculate(right.getCurrentPosition());
        left.set(output_left);
        right.set(output_right);
    }
}
