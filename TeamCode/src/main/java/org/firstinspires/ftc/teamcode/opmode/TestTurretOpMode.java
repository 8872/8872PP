package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public class TestTurretOpMode extends OpMode {
    private ServoEx turret, arm, claw;

    private Claw currentClawState = Claw.OPEN;
    private Control currentControlState = Control.MANUAL;

    public static int down = 180, left = 270, right = 90, up = 0;

    private enum Claw {
        OPEN,
        CLOSED
    }

    private enum Control {
        MANUAL,
        SET
    }

    @Override
    public void init() {
        turret = new SimpleServo(hardwareMap, "turret", 0, 300);
        arm = new SimpleServo(hardwareMap, "arm", 0, 355);
        claw = new SimpleServo(hardwareMap, "claw", 0, 300);
    }

    @Override
    public void loop() {
        if(currentControlState == Control.MANUAL) {
            arm.setPosition((gamepad1.right_stick_y + 1) / 2);
            turret.turnToAngle(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x), AngleUnit.RADIANS);
        } else {
            if(gamepad1.dpad_down) {
                turret.turnToAngle(down);
            }
            if(gamepad1.dpad_left) {
                turret.turnToAngle(left);
            }
            if(gamepad1.dpad_right) {
                turret.turnToAngle(right);
            }
            if(gamepad1.dpad_up) {
                turret.turnToAngle(up);
            }
        }

        if(gamepad1.b && currentControlState == Control.SET) {
            currentControlState = Control.MANUAL;
        } else if(gamepad1.b && currentControlState == Control.MANUAL) {
            currentControlState = Control.SET;
        }

        if(gamepad1.a && currentClawState == Claw.OPEN) {
            claw.setPosition(0);
            currentClawState = Claw.CLOSED;
        } else if(gamepad1.a && currentClawState == Claw.CLOSED) {
            claw.setPosition(1);
            currentClawState = Claw.OPEN;
        }

        telemetry.addData("arm position", arm.getPosition());
        telemetry.addData("turret position", turret.getPosition());
        telemetry.addData("claw position", claw.getPosition());

        telemetry.addData("claw state", currentClawState);
        telemetry.addData("controls state", currentControlState);

        telemetry.update();
    }
}
