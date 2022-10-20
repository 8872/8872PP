package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class BaseOpMode extends CommandOpMode {
    protected MotorEx fL, fR, bL, bR;
    protected DriveSubsystem drive;
    protected RevIMU imu;

    @Override
    public void initialize() {
        initHardware();

        drive = new DriveSubsystem(fL, fR, bL, bR);
        imu = new RevIMU(hardwareMap);
        imu.init();

        composeTelemetry();
        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

    protected void initHardware() {
        fL = hardwareMap.get(MotorEx.class, "leftFront");
        fR = hardwareMap.get(MotorEx.class, "rightFront");
        bL = hardwareMap.get(MotorEx.class, "leftBack");
        bR = hardwareMap.get(MotorEx.class, "rightBack");
    }

    protected void composeTelemetry() {
        telemetry.addData("leftFront Power", () -> round(fL.motor.getPower()));
        telemetry.addData("leftBack Power", () -> round(bL.motor.getPower()));
        telemetry.addData("rightFront Power", () -> round(fR.motor.getPower()));
        telemetry.addData("rightBack Power", () -> round(bR.motor.getPower()));
        telemetry.addData("IMU Heading", () -> imu.getHeading());
    }

    protected void setUpHardwareDevices() {

    }


    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        return new BigDecimal(Double.toString(value)).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    private static double round(double value) {
        return round(value, 4);
    }


}