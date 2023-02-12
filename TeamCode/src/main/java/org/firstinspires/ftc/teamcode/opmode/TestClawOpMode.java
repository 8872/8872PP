package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestClawOpMode extends OpMode {
    private ServoEx claw;

    @Override
    public void init() {
        claw = new SimpleServo(hardwareMap, "claw", 0, 300);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            claw.setPosition(0.5);
        }
        if (gamepad1.b) {
            claw.setPosition(0);
        }

        telemetry.addData("claw position", claw.getPosition());
        telemetry.update();
    }
}
