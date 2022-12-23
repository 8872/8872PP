package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.drive.DefaultFieldCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.DefaultRobotCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.SlowMode;
import org.firstinspires.ftc.teamcode.command.lift.*;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
import org.firstinspires.ftc.teamcode.util.Junction;

@TeleOp(name = "Competition TeleOp")
public final class CompetitionOpMode extends BaseOpMode {
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;


    private DefaultFieldCentricDrive fieldCentricDrive;
    private DefaultRobotCentricDrive robotCentricDrive;
    private SlowMode slowMode;

    private GrabCone grabCone;
    private ReleaseCone releaseCone;

    private SlideIn slideIn;
    private SlideOut slideOut;

    private MoveLiftPID moveLiftPID;
    private SetJunction setJunctionLow, setJunctionMedium, setJunctionHigh, setJunctionGround, setJunctionNone;

    private Button changeCenter, clawStuff, slideStuff, resetEncoders;
    private Button moveHigh, moveMedium, moveLow, moveGround, moveNone;


    @Override
    public void initialize() {
        super.initialize();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        fieldCentricDrive = new DefaultFieldCentricDrive(drive, gamepadEx1::getLeftX, gamepadEx1::getLeftY,
                gamepadEx1::getRightX, imu::getHeading);
        robotCentricDrive = new DefaultRobotCentricDrive(drive, gamepadEx1::getLeftX,
                gamepadEx1::getRightX, gamepadEx1::getLeftY);
        slowMode = new SlowMode(drive, gamepadEx1::getLeftX, gamepadEx1::getRightX, gamepadEx1::getLeftY);

        // no field centric for u :)
        changeCenter = new GamepadButton(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(slowMode, robotCentricDrive);

        clawStuff = new GamepadButton(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new GrabCone(claw), new ReleaseCone(claw));

        slideStuff = (new GamepadButton(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER))
                .toggleWhenPressed(new SlideOut(slide), new SlideIn(slide));

        moveLiftPID = new MoveLiftPID(lift, gamepadEx2::getRightY);

        setJunctionNone = new SetJunction(lift, Junction.NONE);
        setJunctionGround = new SetJunction(lift, Junction.GROUND);
        setJunctionLow = new SetJunction(lift, Junction.LOW);
        setJunctionMedium = new SetJunction(lift, Junction.MEDIUM);
        setJunctionHigh = new SetJunction(lift, Junction.HIGH);

        moveNone = new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(setJunctionNone);
        moveGround = new GamepadButton(gamepadEx2, GamepadKeys.Button.X)
                .whenPressed(setJunctionGround);
        moveLow = new GamepadButton(gamepadEx2, GamepadKeys.Button.A)
                .whenPressed(setJunctionLow);
        moveMedium = new GamepadButton(gamepadEx2, GamepadKeys.Button.B)
                .whenPressed(setJunctionMedium);
        moveHigh = new GamepadButton(gamepadEx2, GamepadKeys.Button.Y)
                .whenPressed(setJunctionHigh);

        register(drive, lift, claw, slide);
        drive.setDefaultCommand(robotCentricDrive);
        lift.setDefaultCommand(moveLiftPID);

    }



}