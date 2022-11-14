package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.LogCatCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

@Config
public class ArmSubsystem extends SubsystemBase {

    private final ServoEx claw, slide;
    private final MotorEx dr4bLeftMotor, dr4bRightMotor;

    public static int LOW = -450;
    public static int MEDIUM = -850;
    public static int HIGH = -1800;
    public static int GROUND = -100;

    // PID coefficients for left dr4b motor
    public static double dr4b_kP = 0.0008; // tune
    public static double dr4b_kI = 0.01;
    public static double dr4b_kD = 0.001; // 0.001
    public static double dr4b_kF = 0;
    private PIDFController dr4b_pidf = new PIDFController(dr4b_kP, dr4b_kI, dr4b_kD, dr4b_kF);
    private double output;
    // enum representing different junction levels
    private enum Junction {
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
        dr4b_pidf.setTolerance(30);
    }

    // grab cone with a certain angle (for tuning)
    public void moveClaw(double pos) {
        claw.rotateBy(pos);
    }

    // grab cone
    public void grab() {
        claw.turnToAngle(100); // determine later
    }

    // release cone
    public void release() {
        claw.turnToAngle(0);
    }

    // move slide to a specified position
    // remove this function?
    public void moveSlide(double pos){
        slide.rotateBy(pos);
    }

    // moves slide to the in most position
    public void slideIn(){
        slide.setPosition(0);
    }

    // moves slide to the out most position
    public void slideOut(){
        slide.setPosition(1);
    }

    public void moveDr4b(double power){
        dr4bRightMotor.set(power / 2);
        dr4bLeftMotor.set(power / 2);
    }

    // moves dr4b to specified position
    private void moveDr4b(Junction junction){
        switch(junction){
            case GROUND:
                dr4b_pidf.setSetPoint(GROUND);
                break;
            case LOW:
                dr4b_pidf.setSetPoint(LOW);
                break;
            case MEDIUM:
                dr4b_pidf.setSetPoint(MEDIUM);
                break;
            case HIGH:
                dr4b_pidf.setSetPoint(HIGH); // tune later
                Log.d("Test", "high");
                break;
        }
        output = dr4b_pidf.calculate(dr4bLeftMotor.getCurrentPosition());
        Log.d("atSetPoint", "" + dr4b_pidf.atSetPoint());
        while(!dr4b_pidf.atSetPoint()){

            output = dr4b_pidf.calculate(dr4bLeftMotor.getCurrentPosition());
            Log.d("output", "" + output);
            Log.d("error", "" + dr4b_pidf.getPositionError());
            Log.d("encoder", "" + dr4bLeftMotor.getCurrentPosition());
            dr4bLeftMotor.set(output);
            dr4bRightMotor.set(output);
        }

        dr4bLeftMotor.stopMotor();
        dr4bRightMotor.stopMotor();

    }

    public double getOutput() {
        return output;
    }

    public double getError(){
        return dr4b_pidf.getPositionError();
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



}
