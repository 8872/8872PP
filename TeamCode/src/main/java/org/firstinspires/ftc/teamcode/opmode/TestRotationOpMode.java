package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.drive.*;

@TeleOp(name = "Test Rotation OpMode")
public final class TestRotationOpMode extends BaseOpMode {
    private GamepadEx gamepadEx1;
    private HeadingPID headingPID;

    @Override
    public void initialize() {
        super.initialize();

        gamepadEx1 = new GamepadEx(gamepad1);

        headingPID = new HeadingPID(drive);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetHeading(drive, 0));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SetHeading(drive, 180));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetHeading(drive, 90));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetHeading(drive, 270));

        register(drive);
        drive.setDefaultCommand(headingPID);


    }



}