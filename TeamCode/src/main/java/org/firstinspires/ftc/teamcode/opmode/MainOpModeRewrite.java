package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.drive.DefaultRobotCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.SlowMode;
import org.firstinspires.ftc.teamcode.command.group.LiftDown;
import org.firstinspires.ftc.teamcode.command.group.LiftUp;
import org.firstinspires.ftc.teamcode.command.lift.*;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
import org.firstinspires.ftc.teamcode.util.Junction;

import java.util.List;

/*
have a list of buttons in an array
and another list of commands
foreach and bind them
for one, you could always just save gamepadEx2.getGamepadButton as a variable
same for GamepadKeys.Button

two(push.X/Y).whenPressed(dumb stuff)

like forEachIndexed { i, button -> gamepad2.getGamepadButton(button).whenPressed(new SetJunction(lift, junctionPositions[i]) }
would work cleanest imo
so ur two arrays would be { dpad_down, x, a } and { none, ground, low }
 */

@TeleOp(name = "test opmode")
public final class MainOpModeRewrite extends BaseOpMode {
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private DefaultRobotCentricDrive robotCentricDrive;
    private SlowMode slowMode;

    private MoveLiftPID moveLiftPID;

    private List<GamepadKeys.Button> oneButtons;
    private List<GamepadKeys.Button> twoButtons;



    @Override
    public void initialize() {
        super.initialize();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robotCentricDrive = new DefaultRobotCentricDrive(drive, gamepadEx1::getLeftX,
                gamepadEx1::getRightX, gamepadEx1::getLeftY);

        slowMode = new SlowMode(drive, gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(slowMode);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new GrabCone(claw), new ReleaseCone(claw));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new SlideOut(slide), new SlideIn(slide));

        moveLiftPID = new MoveLiftPID(lift, gamepadEx2::getRightY);

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SetJunction(lift, Junction.NONE));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SetJunction(lift, Junction.GROUND));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SetJunction(lift, Junction.LOW));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SetJunction(lift, Junction.MEDIUM));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SetJunction(lift, Junction.HIGH));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new LiftUp(lift, slide, Junction.HIGH));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LiftDown(lift, slide, claw));

        register(drive, lift, claw, slide);
        drive.setDefaultCommand(robotCentricDrive);
        lift.setDefaultCommand(moveLiftPID);

    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Target", lift.getTargetPosition());
    }
}