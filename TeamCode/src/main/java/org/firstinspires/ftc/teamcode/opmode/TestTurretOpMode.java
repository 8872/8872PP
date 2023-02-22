package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
public final class TestTurretOpMode extends OpMode {
    private ServoEx turret, arm, claw;
    private ServoImplEx turretEx;
    private AnalogInput turretEnc;

    private ClawState currentClawState = ClawState.OPEN;
    private Control currentControlState = Control.SET;
    private ArmState currentArmState = ArmState.DOWN;

    public static double turretDown = 0, turretLeft = 0.25, turretRight = 0.5, turretUp = 1;
    public static double armDown = 0.95, armDeposit = 0.6;
    public static double clawOpen = 0.3, clawClose = 0.6;
    // turret
    // right forward: 0.88
    // left forward*: 0
    // right back: 0.57
    // left back: 0.29
    // start position: 0.435
    // left: 0.15
    // right: 0.715
    // arm
    // arm down: 1
    // arm deposit: 0.6


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
        turret = new SimpleServo(hardwareMap, "turret", 355, 0);
        arm = new SimpleServo(hardwareMap, "arm", 0, 355);
        claw = new SimpleServo(hardwareMap, "claw", 0, 300);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretEnc = hardwareMap.get(AnalogInput.class, "turretEnc");
        turretEx = hardwareMap.get(ServoImplEx.class, "turret");
//        turretEx.setPwmRange(new PwmControl.PwmRange());

    }

    @Override
    public void loop() {
        if (currentControlState == Control.MANUAL) {
//            arm.setPosition((gamepad1.right_stick_y + 1) / 2);
//            turret.turnToAngle(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x), AngleUnit.RADIANS);
        } else {
            if (gamepad1.dpad_down) {
                turret.setPosition(turretDown);
            }
            if (gamepad1.dpad_left) {
                turret.setPosition(turretLeft);
            }
            if (gamepad1.dpad_right) {
                turret.setPosition(turretRight);
            }
            if (gamepad1.dpad_up) {
                turret.setPosition(turretUp);
            }

            if (gamepad1.a && currentArmState == ArmState.DEPOSIT) {
                currentArmState = ArmState.DOWN;
                arm.setPosition(armDown);
            } else if (gamepad1.a && currentArmState == ArmState.DOWN) {
                currentArmState = ArmState.DEPOSIT;
                arm.setPosition(armDeposit);
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
        tad("turret servo position", turret.getPosition());
        tad("claw position", claw.getPosition());

        tad("claw state", currentClawState);
        tad("control state", currentControlState);
        tad("arm state", currentArmState);

        double turretPosition = (turretEnc.getVoltage() - 0.167) / 2.952 * 355;
        tad("turret encoder voltage / 3.3", turretEnc.getVoltage() / 3.3);
        tad("turret encoder voltage", turretEnc.getVoltage());
        tad("turret encoder position", turretPosition);
        tad("turret servo angle", turret.getAngle());

        telemetry.update();
    }

    private void tad(String caption, Object text) {
        telemetry.addData(caption, text);
    }
}
