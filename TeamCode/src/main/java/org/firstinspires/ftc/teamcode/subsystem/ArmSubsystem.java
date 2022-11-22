package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ArmSubsystem extends SubsystemBase {

    private final ServoEx claw, slide;
    private final DcMotorEx dr4bLeftMotor, dr4bRightMotor;

    public static double kV = 0.0005;
    public static int RESET = 0;
    public static int LOW = -350;
    public static int MEDIUM = -850;
    public static int HIGH = -1750;
    public static int GROUND = -50;

    // PID coefficients for left dr4b motor
    public static double dr4b_kP = 0;
    public static double dr4b_kI = 0;
    public static double dr4b_kD = 0;
    public static double dr4b_kF = 8;
    private double targetPos = 0;
    private final NanoClock clock = NanoClock.system();
    public MotionProfile profile;
    double profileStart;
    public double targetPower = 0;
    // enum representing different junction levels
    public enum Junction {
        RESET,
        GROUND,
        LOW,
        MEDIUM,
        NONE, HIGH
    }

    public ArmSubsystem(ServoEx claw, ServoEx slide, DcMotorEx dr4bLeftMotor, DcMotorEx dr4bRightMotor) {
        this.claw = claw;
        this.slide = slide;
        this.dr4bLeftMotor = dr4bLeftMotor;
        this.dr4bRightMotor = dr4bRightMotor;
        PIDFCoefficients dr4b_coeffs = new PIDFCoefficients(dr4b_kP, dr4b_kI, dr4b_kD, dr4b_kF);
        this.dr4bLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, dr4b_coeffs);
        this.dr4bRightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, dr4b_coeffs);
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
        dr4bRightMotor.setPower(power);
        dr4bLeftMotor.setPower(power);
    }

    private void moveDr4b(Junction junction){}

    public boolean loopPID(){
        double profileTime = clock.seconds() - profileStart;
        if (profileTime > profile.duration()) return false;
        MotionState motionState = profile.get(profileTime);
        targetPower = kV * motionState.getV();
        dr4bRightMotor.setPower(targetPower);
        dr4bLeftMotor.setPower(targetPower);
        return true;
    }

    public void setJunction(Junction junction){
        switch(junction){
            case RESET:
                targetPos = RESET;
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
                break;
        }
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(dr4bLeftMotor.getCurrentPosition(), 0, 0),
                new MotionState(targetPos, 0, 0),
                1000,
                1000
        );
        profileStart = clock.seconds();
    }

    public double getOutput_left() {
        return 0.0;
    }

    public double getError(){
        return 0.0;
        //return dr4b_pidf_left.getPositionError();
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

    public void keepPos(){
        dr4bLeftMotor.setTargetPosition((int) targetPos);
        dr4bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr4bRightMotor.setTargetPosition((int) targetPos);
        dr4bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double getTargetVelocity(){
        if(profile == null) return 0;
        MotionState motionState = profile.get(clock.seconds() - profileStart);
        return motionState.getV();
    }
    public double getLeftVelocity(){
        return dr4bLeftMotor.getVelocity();
    }
    public double getRightVelocity(){
        return dr4bRightMotor.getVelocity();
    }
}
