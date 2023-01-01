package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.drive.DefaultRobotCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.SlowMode;
import org.firstinspires.ftc.teamcode.command.group.GrabAndLift;
import org.firstinspires.ftc.teamcode.command.group.LiftDown;
import org.firstinspires.ftc.teamcode.command.group.LiftUp;
import org.firstinspires.ftc.teamcode.command.lift.*;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
import org.firstinspires.ftc.teamcode.util.ConeStack;
import org.firstinspires.ftc.teamcode.util.Junction;

@Config
@TeleOp(name = "Main TeleOp")
public final class MainOpMode extends BaseOpMode {

    private DefaultRobotCentricDrive robotCentricDrive;
    private SlowMode slowMode;

    private MoveLiftPID moveLiftPID;

    public static int goal = -100;

    @Override
    public void initialize() {
        super.initialize();

        robotCentricDrive = new DefaultRobotCentricDrive(drive, gamepadEx1::getLeftX,
                gamepadEx1::getRightX, gamepadEx1::getLeftY);

        slowMode = new SlowMode(drive, gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY);

        gb1(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(slowMode);

        gb2(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new GrabAndLift(lift, claw, goal), new GrabCone(claw));

        gb2(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new SlideOut(slide), new SlideIn(slide));

        moveLiftPID = new MoveLiftPID(lift, gamepadEx2::getRightY);

        gb2(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new LiftUp(lift, slide, Junction.HIGH));

        gb2(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new LiftUp(lift, slide, Junction.MEDIUM));

        gb2(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LiftUp(lift, slide, Junction.LOW));

        gb2(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new LiftDown(lift, slide, claw));

        gb2(GamepadKeys.Button.Y)
                .whenPressed(new SetConeStack(lift, ConeStack.FIRST));

        gb2(GamepadKeys.Button.X)
                .whenPressed(new SetConeStack(lift, ConeStack.SECOND));

        gb2(GamepadKeys.Button.B)
                .whenPressed(new SetConeStack(lift, ConeStack.THIRD));

        gb2(GamepadKeys.Button.A)
                .whenPressed(new SetConeStack(lift, ConeStack.FOURTH));

        register(drive, lift, claw, slide);
        drive.setDefaultCommand(robotCentricDrive);
        lift.setDefaultCommand(moveLiftPID);

    }

    @Override
    public void run() {
        super.run();
        tad("Target", lift.getTargetPosition());
    }
}