package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.drive.*;

@TeleOp(name = "Test Rotation OpMode")
public final class TestRotationOpMode extends BaseOpMode {
    private HeadingPID headingPID;

    protected TestRotationOpMode(boolean usePhoton, boolean useBulkRead) {
        super(usePhoton, useBulkRead);
    }

    @Override
    public void initialize() {
        super.initialize();

//        headingPID = new HeadingPID(drive);
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SetHeading(drive, 0));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetHeading(drive, 180));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SetHeading(drive, -90));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SetHeading(drive, 90));

        register(drive);
        drive.setDefaultCommand(headingPID);


    }
}