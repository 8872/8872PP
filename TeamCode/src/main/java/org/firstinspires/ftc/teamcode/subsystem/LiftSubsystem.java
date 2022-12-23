package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.util.Junction;

@Config
public class LiftSubsystem extends SubsystemBase {

    private final MotorEx dr4bLeftMotor, dr4bRightMotor;
    private final TouchSensor limitSwitch;

    public static int none = 10;
    public static int low = -450;
    public static int medium = -839;
    public static int high = -1800;
    public static int ground = -25;

    // PID coefficients for left dr4b motor
    public static double dr4b_kP = 0.003;
    public static double dr4b_kI = 0.05;
    public static double dr4b_kD = 0.0003;
    public static double maxVelocity = 2000;
    public static double maxAcceleration = 2000;
    private final ProfiledPIDController dr4b_pidf_left = new ProfiledPIDController(dr4b_kP, dr4b_kI, dr4b_kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    private final ProfiledPIDController dr4b_pidf_right = new ProfiledPIDController(dr4b_kP, dr4b_kI, dr4b_kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
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

    public void loopPID(){
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
                dr4b_pidf_left.setGoal(high); // tune later
                dr4b_pidf_right.setGoal(high);
                break;
        }
    }

    public double getError(){
        return dr4b_pidf_left.getPositionError();
    }

    public void resetEncoders() {
        if(limitSwitch.isPressed()){
            dr4bLeftMotor.resetEncoder();
            dr4bRightMotor.resetEncoder();
        }
    }

    public int getLeftEncoderValue(){
        return dr4bLeftMotor.getCurrentPosition();
    }


    public boolean atTarget() {
        switch(currentGoal){
            case NONE:
                return dr4bLeftMotor.getCurrentPosition()<20 && dr4bLeftMotor.getCurrentPosition()>-20;
            case GROUND:
                return dr4bLeftMotor.getCurrentPosition()<0 && dr4bLeftMotor.getCurrentPosition()>-100;
            case LOW:
                return dr4bLeftMotor.getCurrentPosition()<-300 && dr4bLeftMotor.getCurrentPosition()>-400;
            case MEDIUM:
                return dr4bLeftMotor.getCurrentPosition()<-800 && dr4bLeftMotor.getCurrentPosition()>-900;
            case HIGH:
                return dr4bLeftMotor.getCurrentPosition()<-1700 && dr4bLeftMotor.getCurrentPosition()>-1800;
        }
        return false;
    }

    public void changeSetPoint(double joystickInput) {
        dr4b_pidf_left.setGoal((int) (dr4bLeftMotor.getCurrentPosition()+joystickInput*manualLiftSpeed));
        dr4b_pidf_right.setGoal((int) (dr4bRightMotor.getCurrentPosition()+joystickInput*manualLiftSpeed));
    }

}
