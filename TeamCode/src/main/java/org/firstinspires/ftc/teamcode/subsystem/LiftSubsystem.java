package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.util.ConeStack;
import org.firstinspires.ftc.teamcode.util.Junction;
import org.firstinspires.ftc.teamcode.util.ProfiledPIDFController;

@Config
public class LiftSubsystem extends SubsystemBase {

    private final MotorEx dr4bLeftMotor, dr4bRightMotor;
    private final TouchSensor limitSwitch;

    public static int none = 10;
    public static int low = -400; //-450
    public static int medium = -855; //-839
    public static int high = -1800;
    public static int ground = -25;

    public static int firstCone = -142;
    public static int secondCone = -121;
    public static int thirdCone = -87;
    public static int fourthCone = -60;

    private final double TICKS_IN_DEGREES = 8192.0 / 360;

    // PID coefficients for left dr4b motor
    public static double kP = 0.003;
    public static double kI = 0.05;
    public static double kD = 0.0003;
    public static double kF = 0.07;
    public static double maxVelocity = 2000;
    public static double maxAcceleration = 2000;
    private final ProfiledPIDFController dr4b_pidf_left = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    private final ProfiledPIDFController dr4b_pidf_right = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    private double output_left;
    private double output_right;
    public static double tolerance = 10;
    public static Junction currentGoal = Junction.NONE;
    public static int manualLiftSpeed = 30;

    public LiftSubsystem(MotorEx dr4bLeftMotor, MotorEx dr4bRightMotor, TouchSensor limitSwitch) {
        this.limitSwitch = limitSwitch;
        this.dr4bLeftMotor = dr4bLeftMotor;
        this.dr4bRightMotor = dr4bRightMotor;
        dr4b_pidf_left.setTolerance(tolerance);
        dr4b_pidf_left.setGoal(0);
        dr4b_pidf_right.setGoal(0);
    }

    public void moveDr4b(double power){
        dr4bRightMotor.set(power / 2);
        dr4bLeftMotor.set(power / 2);
    }

    public void updatePID() {
        output_left = dr4b_pidf_left.calculate(dr4bLeftMotor.getCurrentPosition());
        output_right = dr4b_pidf_right.calculate(dr4bRightMotor.getCurrentPosition());
        dr4bLeftMotor.set(output_left);
        dr4bRightMotor.set(output_right);
    }

    public void setJunction(int goal){
        dr4b_pidf_left.setGoal(goal);
        dr4b_pidf_right.setGoal(goal);
    }

    public void setJunction(Junction junction){
        currentGoal = junction;
        switch(junction){
            case NONE:
                dr4b_pidf_left.setGoal(none);
                dr4b_pidf_right.setGoal(none);
                break;
            case GROUND:
                dr4b_pidf_left.setGoal(ground);
                dr4b_pidf_right.setGoal(ground);
                break;
            case LOW:
                dr4b_pidf_left.setGoal(low);
                dr4b_pidf_right.setGoal(low);
                break;
            case MEDIUM:
                dr4b_pidf_left.setGoal(medium);
                dr4b_pidf_right.setGoal(medium);
                break;
            case HIGH:
                dr4b_pidf_left.setGoal(high);
                dr4b_pidf_right.setGoal(high);
                break;
        }
    }

    public void setConeStack(ConeStack cone) {
        switch(cone) {
            case FIRST:
                dr4b_pidf_left.setGoal(firstCone);
                dr4b_pidf_right.setGoal(firstCone);
                break;
            case SECOND:
                dr4b_pidf_left.setGoal(secondCone);
                dr4b_pidf_right.setGoal(secondCone);
                break;
            case THIRD:
                dr4b_pidf_left.setGoal(thirdCone);
                dr4b_pidf_right.setGoal(thirdCone);
                break;
            case FOURTH:
                dr4b_pidf_left.setGoal(fourthCone);
                dr4b_pidf_right.setGoal(fourthCone);
                break;
        }
    }

    public void checkLimitSwitch() {
        if (limitSwitch.isPressed()) {
            dr4bLeftMotor.resetEncoder();
            dr4bRightMotor.resetEncoder();
        }
    }

    public int getLeftEncoderValue(){
        return dr4bLeftMotor.getCurrentPosition();
    }


    public boolean atTarget(int goal){
        return Math.abs(dr4bLeftMotor.getCurrentPosition()-goal)<60;
    }
    public boolean atTarget() {
        switch(currentGoal){
            case NONE:
                return dr4bLeftMotor.getCurrentPosition()<20 && dr4bLeftMotor.getCurrentPosition()>-20;
            case GROUND:
                return dr4bLeftMotor.getCurrentPosition()<0 && dr4bLeftMotor.getCurrentPosition()>-100;
            case LOW:
                return dr4bLeftMotor.getCurrentPosition()<-300 && dr4bLeftMotor.getCurrentPosition()>-501;
            case MEDIUM:
                return dr4bLeftMotor.getCurrentPosition()<-800 && dr4bLeftMotor.getCurrentPosition()>-900;
            case HIGH:
                return dr4bLeftMotor.getCurrentPosition()<-1700 && dr4bLeftMotor.getCurrentPosition()>-1800;
        }
        return false;
    }

    public void setCurrentGoal(Junction junction){
        currentGoal = junction;
    }

    public void changeSetPoint(double joystickInput) {
        dr4b_pidf_left.setGoal((int) (dr4bLeftMotor.getCurrentPosition()+joystickInput*manualLiftSpeed));
        dr4b_pidf_right.setGoal((int) (dr4bRightMotor.getCurrentPosition()+joystickInput*manualLiftSpeed));
    }

    public int getTargetPosition(){
        return (int) dr4b_pidf_left.getGoal().position;
    }

    public Junction getCurrentGoal() {
        return currentGoal;
    }

    public boolean isSlideIncompatible(){
        return currentGoal == Junction.LOW || currentGoal == Junction.GROUND;
    }

    public boolean isGrabAndLiftIncompatible(){
        return currentGoal != Junction.NONE;
    }
}
