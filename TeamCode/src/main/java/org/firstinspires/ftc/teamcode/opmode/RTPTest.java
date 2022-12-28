package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Disabled
@TeleOp(name = "RTP Test")
public class RTPTest extends OpMode {

    public static int none = 10;
    public static int low = -450;
    public static int medium = -839;
    public static int high = -1800;
    public static int ground = -25;

    private DcMotorEx dr4bLeftMotor, dr4bRightMotor;


    @Override
    public void init() {
        dr4bLeftMotor = hardwareMap.get(DcMotorEx.class, "dr4bLeft");
        dr4bRightMotor = hardwareMap.get(DcMotorEx.class, "dr4bRight");
        dr4bLeftMotor.setTargetPosition(0);
        dr4bRightMotor.setTargetPosition(0);
        dr4bLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dr4bRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dr4bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr4bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dr4bRightMotor.setPower(1);
        dr4bLeftMotor.setPower(1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_down) {
            dr4bLeftMotor.setTargetPosition(none);
        } else if (gamepad1.dpad_up) {
            dr4bLeftMotor.setTargetPosition(high);
        } else if (gamepad1.dpad_left) {
            dr4bLeftMotor.setTargetPosition(medium);
        } else if (gamepad1.dpad_right) {
            dr4bLeftMotor.setTargetPosition(low);
        }

        telemetry.addData("dr4bLeftMotor Position", dr4bLeftMotor.getCurrentPosition());
        telemetry.addData("dr4bRightMotor Position", dr4bRightMotor.getCurrentPosition());

        telemetry.addData("dr4bLeftMotor Power", dr4bLeftMotor.getPower());
        telemetry.addData("dr4bRightMotor Power", dr4bRightMotor.getPower());

        telemetry.addData("Target", dr4bLeftMotor.getTargetPosition());
        telemetry.update();
    }
}
