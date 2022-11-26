package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class ArmSubsystem extends SubsystemBase {

    private final ServoEx claw, slide;
    private final MotorEx dr4bLeftMotor, dr4bRightMotor;
    private final TouchSensor limitSwitch;

    public static int NONE = 10;
    public static int LOW = -336;
    public static int MEDIUM = -839;
    public static int HIGH = -1750;
    public static int GROUND = -25;

    // PID coefficients for left dr4b motor
    public static double dr4b_kP = 0.003;
    public static double dr4b_kI = 0.05;
    public static double dr4b_kD = 0.0003;
    public static double maxVelocity = 1000;
    public static double maxAcceleration = 1000;
    private final ProfiledPIDController dr4b_pidf_left = new ProfiledPIDController(dr4b_kP, dr4b_kI, dr4b_kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    private final ProfiledPIDController dr4b_pidf_right = new ProfiledPIDController(dr4b_kP, dr4b_kI, dr4b_kD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    private double output_left;
    private double output_right;
    public static double tolerance = 10;
    public static Junction currentGoal = Junction.NONE;
    public static double slideGoal = 1.0;
    public static double slideInPos = 1.0;
    public static double slideOutPos = 0.0;
    public static double clawGrabPos = 25;
    public static double clawReleasePos = 85;
    // enum representing different junction levels
    public enum Junction {
        NONE,
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    public ArmSubsystem(ServoEx claw, ServoEx slide, MotorEx dr4bLeftMotor, MotorEx dr4bRightMotor, TouchSensor limitSwitch) {
        this.claw = claw;
        this.slide = slide;
        this.limitSwitch = limitSwitch;
        this.dr4bLeftMotor = dr4bLeftMotor;
        this.dr4bRightMotor = dr4bRightMotor;
        dr4b_pidf_left.setTolerance(tolerance);
        dr4b_pidf_left.setGoal(0);
        dr4b_pidf_right.setGoal(0);
    }

    // grab cone with a certain angle (for tuning)
    public void moveClaw(double pos) {
        claw.rotateBy(pos);
    }

    // grab cone
    public void grab() {
        claw.turnToAngle(clawGrabPos); // determine later
    }

    // release cone
    public void release() {
        claw.turnToAngle(clawReleasePos);
    }

    // move slide to a specified position
    // remove this function?
    public void moveSlide(double pos){
        slide.rotateBy(pos);
    }

    // moves slide to the in most position
    public void slideIn(){
        slideGoal = 1.0;
        slide.setPosition(slideInPos);
    }

    // moves slide to the out most position
    public void slideOut(){
        slideGoal = 0.0;
        slide.setPosition(slideOutPos);
    }

    public void moveDr4b(double power){
        dr4bRightMotor.set(power / 2);
        dr4bLeftMotor.set(power / 2);
    }

    // moves dr4b to specified position
    private void moveDr4b(Junction junction){
        switch(junction){
            case GROUND:
                dr4b_pidf_left.setGoal(GROUND);
                dr4b_pidf_right.setGoal(GROUND);
                break;
            case LOW:
                dr4b_pidf_left.setGoal(LOW);
                dr4b_pidf_right.setGoal(LOW);
                break;
            case MEDIUM:
                dr4b_pidf_left.setGoal(MEDIUM);
                dr4b_pidf_right.setGoal(MEDIUM);
                break;
            case HIGH:
                dr4b_pidf_left.setGoal(HIGH); // tune later
                dr4b_pidf_right.setGoal(HIGH);
                Log.d("Test", "high");
                break;
        }
        output_left = dr4b_pidf_left.calculate(dr4bLeftMotor.getCurrentPosition());
        Log.d("atSetPoint", "" + dr4b_pidf_left.atGoal());
        while(!dr4b_pidf_left.atGoal()){

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

    private void moveDr4b(Junction junction, boolean testMotionProfiling){
        com.acmerobotics.roadrunner.control.PIDFController controller = new com.acmerobotics.roadrunner.control.PIDFController(new PIDCoefficients(dr4b_kP, dr4b_kI, dr4b_kD));

        int targetPos = 0;
        switch(junction){
            case GROUND:
                targetPos = GROUND;
                break;
            case LOW:
                targetPos = LOW;
                break;
            case MEDIUM:
                targetPos = MEDIUM;
                break;
            case HIGH:
                targetPos = HIGH;
                Log.d("Test", "high");
                break;
        }

        NanoClock clock = NanoClock.system();


        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(targetPos, 0, 0),
                2000,
                2000,
                1000
        );

        double profileStart = clock.seconds();

        while(true){
            double profileTime = clock.seconds() - profileStart;
            if (profileTime > profile.duration()) {
                break;
            }

            MotionState motionState = profile.get(profileTime);

            controller.setTargetPosition(motionState.getX());
            controller.setTargetVelocity(motionState.getV());
            controller.setTargetAcceleration(motionState.getA());

            controller.update(dr4bLeftMotor.getCurrentPosition());



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

    public void setJunction(int goal){
        dr4b_pidf_left.setGoal(goal);
        dr4b_pidf_right.setGoal(goal);
    }

    public void setJunction(Junction junction){
        currentGoal = junction;
        switch(junction){
            case NONE:
                dr4b_pidf_left.setGoal(NONE);
                dr4b_pidf_right.setGoal(NONE);
                break;
            case GROUND:
                dr4b_pidf_left.setGoal(GROUND);
                dr4b_pidf_right.setGoal(GROUND);
                break;
            case LOW:
                dr4b_pidf_left.setGoal(LOW);
                dr4b_pidf_right.setGoal(LOW);
                break;
            case MEDIUM:
                dr4b_pidf_left.setGoal(MEDIUM);
                dr4b_pidf_right.setGoal(MEDIUM);
                break;
            case HIGH:
                dr4b_pidf_left.setGoal(HIGH); // tune later
                dr4b_pidf_right.setGoal(HIGH);
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

    public void resetEncoders(){
        if(limitSwitch.isPressed()){
            dr4bLeftMotor.resetEncoder();
            dr4bRightMotor.resetEncoder();
        }
    }
    public int getLeftEncoderValue(){
        return dr4bLeftMotor.getCurrentPosition();
    }

    public double getSlidePos(){
        return slide.getPosition();
    }

    public boolean atTarget(){
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

    public boolean slidePosReached(){
        return Math.abs(slide.getPosition()-slideGoal)<0.2;
    }
}
