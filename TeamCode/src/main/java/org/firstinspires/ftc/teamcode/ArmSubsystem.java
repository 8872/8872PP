package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {

    private final ServoEx claw, slide;
    private final MotorEx dr4bLeft, dr4bRight;

    // PID coefficients for left dr4b motor
    private final static double dr4b_left_kP = 1;
    private final static double dr4b_left_kI = 0;
    private final static double dr4b_left_kD = 0;

    // PID coefficients for right dr4b motor
    private final static double dr4b_right_kP = 1;
    private final static double dr4b_right_kI = 0;
    private final static double dr4b_right_kD = 0;

    private enum Junction {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }


    public ArmSubsystem(ServoEx claw, ServoEx slide, MotorEx dr4bLeft, MotorEx dr4bRight) {
        this.claw = claw;
        this.slide = slide;
        this.dr4bLeft = dr4bLeft;
        this.dr4bRight = dr4bRight;
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
    public void moveSlide(double pos){
        slide.setPosition(pos);
    }

    // moves slide to the in most position
    public void slideIn(){
        slide.setPosition(0);
    }

    // moves slide to the out most position
    public void slideOut(){
        slide.setPosition(1);
    }

    // moves dr4b to specified position
    private void moveDr4b(Junction junct){
        dr4bLeft.setRunMode(Motor.RunMode.VelocityControl);
        dr4bRight.setRunMode(Motor.RunMode.VelocityControl);

        // pid loop for motors

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
