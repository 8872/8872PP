package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmSubsystem extends SubsystemBase {

    private final ServoEx claw, slide1, slide2;
    private final MotorEx dr4bLeftMotor, dr4bRightMotor;

    // PID coefficients for left dr4b motor
    private final static double left_kP = 1; // tune
    private final static double left_kI = 0;
    private final static double left_kD = 0;
    private final static double left_kF = 0;
    private PIDFController left_pidf = new PIDFController(left_kP, left_kI, left_kD, left_kF);

    // PID coefficients for right dr4b motor
    private final static double right_kP = 1; // tune
    private final static double right_kI = 0;
    private final static double right_kD = 0;
    private final static double right_kF = 0;
    private PIDFController right_pidf = new PIDFController(right_kP, right_kI, right_kD, right_kF);
    // enum representing different junction levels
    private enum Junction {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    public ArmSubsystem(ServoEx claw, ServoEx slide1, ServoEx slide2 , MotorEx dr4bLeftMotor, MotorEx dr4bRightMotor) {
        this.claw = claw;
        this.slide1 = slide1;
        this.slide2 = slide2;
        this.dr4bLeftMotor = dr4bLeftMotor;
        this.dr4bRightMotor = dr4bLeftMotor;

        dr4bLeftMotor.setRunMode(Motor.RunMode.VelocityControl);
        dr4bRightMotor.setRunMode(Motor.RunMode.VelocityControl);
        // brake ;-; ?
        dr4bLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dr4bRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

    }

    // grab cone with a certain angle (for tuning)
    public void grab(double angle) {
        claw.turnToAngle(angle);
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
        slide1.setPosition(pos);
    }

    // moves slide to the in most position
    public void slideIn(){
        slide1.setPosition(0);
        slide2.setPosition(0);
    }

    // moves slide to the out most position
    public void slideOut(){
        slide1.setPosition(1);
        slide2.setPosition(1);
    }

    // moves dr4b to specified position
    // rewrite
    private void moveDr4b(Junction junction){
        switch(junction){
            case GROUND:
                left_pidf.setSetPoint(0); // to be tuned later
                right_pidf.setSetPoint(0);
                break;
            case LOW:
                left_pidf.setSetPoint(20);
                right_pidf.setSetPoint(20);
                break;
            case MEDIUM:
                left_pidf.setSetPoint(40);
                right_pidf.setSetPoint(40);
                break;
            case HIGH:
                left_pidf.setSetPoint(60);
                right_pidf.setSetPoint(60);
                break;
        }

        // control loop for pid
        while(!left_pidf.atSetPoint() && !right_pidf.atSetPoint()){
            double outputLeft = left_pidf.calculate(dr4bLeftMotor.getCurrentPosition());
            double outputRight = right_pidf.calculate(dr4bRightMotor.getCurrentPosition());
            dr4bLeftMotor.setVelocity(outputLeft); // setPower() instead?
            dr4bRightMotor.setVelocity(outputRight);
        }

        // stop motors; should break.
        dr4bLeftMotor.stopMotor();
        dr4bRightMotor.stopMotor();
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
