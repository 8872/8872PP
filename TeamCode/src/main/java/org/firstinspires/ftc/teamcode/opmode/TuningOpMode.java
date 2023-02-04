package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.PIDFControllerEx;

//@Disabled
@Config
@TeleOp(name = "Tuning OpMode")
public final class TuningOpMode extends OpMode {

    public static int none = 10;
    public static int low = -450;
    public static int medium = -839;
    public static int high = -1800;
    public static int ground = -25;

    private final double TICKS_IN_DEGREES = 8192.0 / 360;

    // PID coefficients for left dr4b motor
    public static double kP = 0.003;
    public static double kI = 0.05;
    public static double kD = 0.0003;
    public static double kF = 0.05;
    private PIDFControllerEx left_controller;
    private PIDFControllerEx right_controller;
    private double output_left;
    private double output_right;

    private MotorEx dr4bLeftMotor, dr4bRightMotor;


    @Override
    public void init() {
        dr4bLeftMotor = new MotorEx(hardwareMap, "dr4bLeft");
        dr4bRightMotor = new MotorEx(hardwareMap, "dr4bRight");
        dr4bLeftMotor.setRunMode(Motor.RunMode.RawPower);
        dr4bRightMotor.setRunMode(Motor.RunMode.RawPower);
        dr4bLeftMotor.resetEncoder();
        dr4bRightMotor.resetEncoder();

        left_controller = new PIDFControllerEx(kP, kI, kD, kF, TICKS_IN_DEGREES);
        right_controller = new PIDFControllerEx(kP, kI, kD, kF, TICKS_IN_DEGREES);
        left_controller.setSetPoint(0);
        right_controller.setSetPoint(0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        output_left = left_controller.calculate(dr4bLeftMotor.getCurrentPosition());
        output_right = right_controller.calculate(dr4bRightMotor.getCurrentPosition());

        if (gamepad1.dpad_down) {
            setGoal(none);
        } else if (gamepad1.dpad_up) {
            setGoal(high);
        } else if (gamepad1.dpad_left) {
            setGoal(medium);
        } else if (gamepad1.dpad_right) {
            setGoal(low);
        }

        dr4bLeftMotor.set(output_left);
        dr4bRightMotor.set(output_right);


        telemetry.addData("dr4bLeftMotor Position", dr4bLeftMotor.getCurrentPosition());
        telemetry.addData("dr4bRightMotor Position", dr4bRightMotor.getCurrentPosition());

        telemetry.addData("output_left", output_left);
        telemetry.addData("output_right", output_right);

        telemetry.addData("dr4bLeftMotor Power", dr4bLeftMotor.get());
        telemetry.addData("dr4bRightMotor Power", dr4bRightMotor.get());

        telemetry.addData("Target", left_controller.getSetPoint());
        telemetry.update();
    }

    private void setGoal(int goal) {
        left_controller.setSetPoint(goal);
        right_controller.setSetPoint(goal);
    }
}
