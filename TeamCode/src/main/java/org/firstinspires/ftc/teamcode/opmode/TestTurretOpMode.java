package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public final class TestTurretOpMode extends OpMode {
    private ServoEx turret, arm, claw;

    private ClawState currentClawState = ClawState.OPEN;
    private Control currentControlState = Control.MANUAL;
    private ArmState currentArmState = ArmState.DOWN;

    public static int turretDown = 180, turretLeft = 270, turretRight = 90, turretUp = 0;
    public static int armDown = 0, armDeposit = 90;
    public static double clawOpen = 0.3, clawClose = 0.6;


    private enum ClawState {
        OPEN,
        CLOSED
    }

    private enum Control {
        MANUAL,
        SET
    }

    private enum ArmState {
        DOWN,
        DEPOSIT
    }


    @Override
    public void init() {
        turret = new SimpleServo(hardwareMap, "turret", 0, 300);
        arm = new SimpleServo(hardwareMap, "arm", 0, 355);
        claw = new SimpleServo(hardwareMap, "claw", 0, 300);
    }

    @Override
    public void loop() {
        if (currentControlState == Control.MANUAL) {
            arm.setPosition((gamepad1.right_stick_y + 1) / 2);
            turret.turnToAngle(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x), AngleUnit.RADIANS);
        } else {
            if (gamepad1.dpad_down) {
                turret.turnToAngle(turretDown);
            }
            if (gamepad1.dpad_left) {
                turret.turnToAngle(turretLeft);
            }
            if (gamepad1.dpad_right) {
                turret.turnToAngle(turretRight);
            }
            if (gamepad1.dpad_up) {
                turret.turnToAngle(turretUp);
            }

            if (gamepad1.a && currentArmState == ArmState.DEPOSIT) {
                currentArmState = ArmState.DOWN;
                arm.turnToAngle(armDown);
            } else if (gamepad1.a && currentArmState == ArmState.DOWN) {
                currentArmState = ArmState.DEPOSIT;
                arm.turnToAngle(armDeposit);
            }

        }

        if (gamepad1.left_bumper && currentControlState == Control.SET) {
            currentControlState = Control.MANUAL;
        } else if (gamepad1.left_bumper && currentControlState == Control.MANUAL) {
            currentControlState = Control.SET;
        }

        if (gamepad1.right_bumper && currentClawState == ClawState.OPEN) {
            currentClawState = ClawState.CLOSED;
            claw.setPosition(clawOpen);
        } else if (gamepad1.right_bumper && currentClawState == ClawState.CLOSED) {
            currentClawState = ClawState.OPEN;
            claw.setPosition(clawClose);
        }

        tad("arm position", arm.getPosition());
        tad("turret position", turret.getPosition());
        tad("claw position", claw.getPosition());

        tad("claw state", currentClawState);
        tad("control state", currentControlState);
        tad("arm state", currentArmState);

        telemetry.update();
    }

    private void tad(String caption, Object text) {
        telemetry.addData(caption, text);
    }
}
