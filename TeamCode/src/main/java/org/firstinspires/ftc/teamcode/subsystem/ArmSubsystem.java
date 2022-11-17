package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

@Config
public class ArmSubsystem extends SubsystemBase {

    private final ServoEx claw, slide;
    private final MotorEx dr4bLeftMotor, dr4bRightMotor;

    public static int LOW = -350;
    public static int MEDIUM = -850;
    public static int HIGH = -1750;
    public static int GROUND = -100;

    public static int clawMin = 60;
    public static int clawMax = 90;


    // PID coefficients for left dr4b motor
    public static double dr4b_kP = 0.001;
    public static double dr4b_kI = 0;
    public static double dr4b_kD = 0.0001;
    public static double dr4b_kF = 0;
    private PIDFController dr4b_pidf_left = new PIDFController(dr4b_kP, dr4b_kI, dr4b_kD, dr4b_kF);
    private PIDFController dr4b_pidf_right = new PIDFController(dr4b_kP, dr4b_kI, dr4b_kD, dr4b_kF);
    private double output_left;
    private double output_right;
    // enum representing different junction levels
    public enum Junction {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    public ArmSubsystem(ServoEx claw, ServoEx slide, MotorEx dr4bLeftMotor, MotorEx dr4bRightMotor) {
        this.claw = claw;
        this.slide = slide;
        this.dr4bLeftMotor = dr4bLeftMotor;
        this.dr4bRightMotor = dr4bRightMotor;
        dr4b_pidf_left.setTolerance(30);
        dr4b_pidf_left.setSetPoint(0); // temporary
    }

    // grab cone with a certain angle (for tuning)
    public void moveClaw(double pos) {
        claw.rotateBy(pos);
    }

    // grab cone
    public void grab() {
        claw.turnToAngle(clawMax); // determine later
    }

    // release cone
    public void release() {
        claw.turnToAngle(clawMin);
    }

    // move slide to a specified position
    // remove this function?
    public void moveSlide(double pos){
        slide.rotateBy(pos);
    }

    // moves slide to the in most position
    public void slideIn(){
        slide.setPosition(0.1);
    }

    // moves slide to the out most position
    public void slideOut(){
        slide.setPosition(0.9);
    }

    public void moveDr4b(double power){
        dr4bRightMotor.set(power / 2);
        dr4bLeftMotor.set(power / 2);
    }

    // moves dr4b to specified position
    private void moveDr4b(Junction junction){
        switch(junction){
            case GROUND:
                dr4b_pidf_left.setSetPoint(GROUND);
                dr4b_pidf_right.setSetPoint(GROUND);
                break;
            case LOW:
                dr4b_pidf_left.setSetPoint(LOW);
                dr4b_pidf_right.setSetPoint(LOW);
                break;
            case MEDIUM:
                dr4b_pidf_left.setSetPoint(MEDIUM);
                dr4b_pidf_right.setSetPoint(MEDIUM);
                break;
            case HIGH:
                dr4b_pidf_left.setSetPoint(HIGH); // tune later
                dr4b_pidf_right.setSetPoint(HIGH);
                Log.d("Test", "high");
                break;
        }
        output_left = dr4b_pidf_left.calculate(dr4bLeftMotor.getCurrentPosition());
        Log.d("atSetPoint", "" + dr4b_pidf_left.atSetPoint());
        while(!dr4b_pidf_left.atSetPoint()){

            output_left = dr4b_pidf_left.calculate(dr4bLeftMotor.getCurrentPosition());
            Log.d("output", "" + output_left);
            Log.d("error", "" + dr4b_pidf_left.getPositionError());
            Log.d("encoder", "" + dr4bLeftMotor.getCurrentPosition());
            dr4bLeftMotor.set(output_left);
            dr4bRightMotor.set(output_left);
        }

        dr4bLeftMotor.stopMotor();
        dr4bRightMotor.stopMotor();

    }

    public void loopPID(){
        output_left = dr4b_pidf_left.calculate(dr4bLeftMotor.getCurrentPosition());
        output_right = dr4b_pidf_right.calculate(dr4bRightMotor.getCurrentPosition());
        dr4bLeftMotor.set(output_left);
        dr4bRightMotor.set(output_right);

    }

    public void setJunction(Junction junction){
        switch(junction){
            case GROUND:
                dr4b_pidf_left.setSetPoint(GROUND);
                dr4b_pidf_right.setSetPoint(GROUND);
                break;
            case LOW:
                dr4b_pidf_left.setSetPoint(LOW);
                dr4b_pidf_right.setSetPoint(LOW);
                break;
            case MEDIUM:
                dr4b_pidf_left.setSetPoint(MEDIUM);
                dr4b_pidf_right.setSetPoint(MEDIUM);
                break;
            case HIGH:
                dr4b_pidf_left.setSetPoint(HIGH); // tune later
                dr4b_pidf_right.setSetPoint(HIGH);
                break;
        }
    }

    public double getOutput_left() {
        return output_left;
    }

    public double getError(){
        return dr4b_pidf_left.getPositionError();
    }

    // move dr4b to ground
    public void dr4bGround(){
         moveDr4b(Junction.GROUND);
    }

    // move dr4b to low junction
    public void dr4bLow(){
        moveDr4b(Junction.LOW);
    }

    // move dr4b to medium junction
    public void dr4bMedium(){
        moveDr4b(Junction.MEDIUM);
    }

    // move dr4b to high junction
    public void dr4bHigh(){
        moveDr4b(Junction.HIGH);
    }


    double motion_profile(double max_acceleration, double max_velocity, double distance, double current_dt) {

        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.

        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deacceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deacceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deacceleration_dt;
        if (current_dt > entire_dt)
            return distance;

        // if we're accelerating
        if (current_dt < acceleration_dt)
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(current_dt, 2);

            // if we're cruising
        else if (current_dt < deacceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = current_dt - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deacceleration_time = current_dt - deacceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deacceleration_time - 0.5 * max_acceleration * Math.pow(deacceleration_time, 2);
        }
    }

}
