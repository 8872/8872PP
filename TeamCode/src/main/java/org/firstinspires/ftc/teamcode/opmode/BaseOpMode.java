package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class BaseOpMode extends CommandOpMode {
    protected MotorEx fL, fR, bL, bR, dr4bLeftMotor, dr4bRightMotor;
    protected SimpleServo claw, slide;
    protected DriveSubsystem drive;
    protected ArmSubsystem arm;
    protected RevIMU imu;
    protected TouchSensor limitSwitch;

    @Override
    public void initialize() {
        initHardware();
        setUpHardwareDevices();
        drive = new DriveSubsystem(fL, fR, bL, bR);
        arm = new ArmSubsystem(claw, slide, dr4bLeftMotor, dr4bRightMotor, limitSwitch);
        imu = new RevIMU(hardwareMap);
        imu.init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

    protected void initHardware() {
        fL = new MotorEx(hardwareMap, "leftFront");
        fR = new MotorEx(hardwareMap, "rightFront");
        bL = new MotorEx(hardwareMap, "leftBack");
        bR = new MotorEx(hardwareMap, "rightBack");
        dr4bLeftMotor = new MotorEx(hardwareMap, "dr4bLeft");
        dr4bRightMotor = new MotorEx(hardwareMap, "dr4bRight");
        // what the proper min and max?
        claw = new SimpleServo(hardwareMap, "claw", 0, 120);
        slide = new SimpleServo(hardwareMap, "slide", 0, 120);
        limitSwitch = hardwareMap.get(TouchSensor.class, "touch");
//        slide2 = new SimpleServo(hardwareMap, "slide2", 0, 120);
        dr4bLeftMotor.resetEncoder();
        dr4bRightMotor.resetEncoder();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("leftFront Power", round(fL.motor.getPower()));
        telemetry.addData("leftBack Power", round(bL.motor.getPower()));
        telemetry.addData("rightFront Power", round(fR.motor.getPower()));
        telemetry.addData("rightBack Power", round(bR.motor.getPower()));
        telemetry.addData("dr4bLeftMotor Power", round(dr4bLeftMotor.motor.getPower()));
        telemetry.addData("dr4bRightMotor Power", round(dr4bRightMotor.motor.getPower()));
        telemetry.addData("dr4bRightMotor encoder", dr4bRightMotor.getCurrentPosition());
        telemetry.addData("dr4bLeftMotor encoder", dr4bLeftMotor.getCurrentPosition());

        telemetry.addData("pid output", arm.getOutput_left());
        telemetry.addData("position error", arm.getError());

        telemetry.addData("claw Position", claw.getPosition());
        telemetry.addData("slide1 Position", claw.getPosition());

        telemetry.addData("IMU Heading", imu.getHeading());

        telemetry.addData("limit pressed", limitSwitch.isPressed());
        telemetry.update();
    }

    protected void setUpHardwareDevices() {
        // reverse motors
        //fR.setInverted(true);
        //bR.setInverted(true);
        fL.setInverted(true);
        bL.setInverted(true);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        dr4bLeftMotor.setRunMode(Motor.RunMode.RawPower);
        dr4bRightMotor.setRunMode(Motor.RunMode.RawPower);
        // brake ;-; ????
        dr4bLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dr4bRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }


    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        return new BigDecimal(Double.toString(value)).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    private static double round(double value) {
        return round(value, 4);
    }


}
