package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class ArmSubsystem extends SubsystemBase {

    private final ServoEx claw, slide;
    private final MotorEx dr4bLeftMotor, dr4bRightMotor;

    // PID coefficients for left dr4b motor
    private final static double dr4b_kP = 1; // tune
    private final static double dr4b_kI = 0;
    private final static double dr4b_kD = 0;
    private final static double dr4b_kF = 0;
    private PIDFController dr4b_pidf = new PIDFController(dr4b_kP, dr4b_kI, dr4b_kD, dr4b_kF);

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
        this.dr4bRightMotor = dr4bLeftMotor;

        dr4bLeftMotor.setRunMode(Motor.RunMode.RawPower);
        dr4bRightMotor.setRunMode(Motor.RunMode.RawPower);
        // brake ;-; ????
        dr4bLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dr4bRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

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
        dr4bLeftMotor.set(power);
        dr4bRightMotor.set(power);
    }

    // moves dr4b to specified position
    private void moveDr4b(Junction junction){
        switch(junction){
            case GROUND:
                dr4b_pidf.setSetPoint(0);
                break;
            case LOW:
                dr4b_pidf.setSetPoint(20);
                break;
            case MEDIUM:
                dr4b_pidf.setSetPoint(40);
                break;
            case HIGH:
                dr4b_pidf.setSetPoint(60); // tune later
                break;
        }

        while(dr4b_pidf.atSetPoint()){
            double output = dr4b_pidf.calculate((dr4bLeftMotor.getCurrentPosition()
                    + dr4bRightMotor.getCurrentPosition()) / 2.0);
            dr4bLeftMotor.set(output);
            dr4bRightMotor.set(output);
        }
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
