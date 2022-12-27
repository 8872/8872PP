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
import org.firstinspires.ftc.teamcode.util.ConeStack;
import org.firstinspires.ftc.teamcode.util.Junction;

@TeleOp(name = "final opmode ig pls work ;-;")
public class CompetitionOpMode extends BaseOpMode {
    private GamepadEx driverOp1;
    private GamepadEx driverOp2;

    private DefaultFieldCentricDrive fieldCentricDrive;
    private DefaultRobotCentricDrive robotCentricDrive;
    private SlowMode slowMode;

    private ManualLift manualLift;
    private GrabCone grabCone;
    private ReleaseCone releaseCone;

    private SlideIn slideIn;
    private SlideOut slideOut;

    private MoveLiftPID moveLiftPID;
    private SetJunction setJunctionLow, setJunctionMedium, setJunctionHigh, setJunctionGround, setJunctionNone;
    private SetConeStack setFirst, setSecond, setThird, setFourth;

    private Button changeCenter, clawStuff, slideStuff, resetEncoders;
    private Button moveHigh, moveMedium, moveLow, moveGround, moveNone, moveFirst, moveSecond, moveThird, moveFourth;


    @Override
    public void initialize() {
        super.initialize();

        driverOp1 = new GamepadEx(gamepad1);
        driverOp2 = new GamepadEx(gamepad2);

        fieldCentricDrive = new DefaultFieldCentricDrive(drive, () -> driverOp1.getLeftX(),
                () -> driverOp1.getLeftY(), () -> driverOp1.getRightX(), () -> imu.getHeading());
        robotCentricDrive = new DefaultRobotCentricDrive(drive, () -> driverOp1.getLeftX(),
                () -> driverOp1.getRightX(), () -> driverOp1.getLeftY());
        slowMode = new SlowMode(drive, () -> driverOp1.getLeftX(),
                () -> driverOp1.getRightX(), () -> driverOp1.getLeftY());

        // no field centric for u :)
        changeCenter = (new GamepadButton(driverOp1, GamepadKeys.Button.LEFT_BUMPER)).
                toggleWhenPressed(slowMode, robotCentricDrive);


        grabCone = new GrabCone(claw);
        releaseCone = new ReleaseCone(claw);
        clawStuff = (new GamepadButton(driverOp2, GamepadKeys.Button.LEFT_BUMPER))
                .toggleWhenPressed(grabCone, releaseCone);

        slideIn = new SlideIn(slide);
        slideOut = new SlideOut(slide);
        slideStuff = (new GamepadButton(driverOp2, GamepadKeys.Button.RIGHT_BUMPER))
                .toggleWhenPressed(slideOut, slideIn);

//        moveSlide = new MoveSlide(arm, () -> driverOp2.getLeftY());
//        moveClaw = new MoveClaw(arm, () -> driverOp2.getRightX());


//        resetEncoders = (new GamepadButton(driverOp2, GamepadKeys.Button.A)).whenPressed(
//                new InstantCommand(() -> {
//                    dr4bLeftMotor.resetEncoder();
//                    dr4bRightMotor.resetEncoder();
//                }, arm));
//

        moveLiftPID = new MoveLiftPID(lift, () -> driverOp2.getRightY());

        setJunctionNone = new SetJunction(lift, Junction.NONE);
        setJunctionGround = new SetJunction(lift, Junction.GROUND);
        setJunctionLow = new SetJunction(lift, Junction.LOW);
        setJunctionMedium = new SetJunction(lift, Junction.MEDIUM);
        setJunctionHigh = new SetJunction(lift, Junction.HIGH);
        setFirst = new SetConeStack(lift, ConeStack.FIRST);
        setSecond = new SetConeStack(lift, ConeStack.SECOND);
        setThird = new SetConeStack(lift, ConeStack.THIRD);
        setFourth = new SetConeStack(lift, ConeStack.FOURTH);

        moveGround = (new GamepadButton(driverOp2, GamepadKeys.Button.X))
                .whenPressed(setJunctionGround);
        moveLow = (new GamepadButton(driverOp2, GamepadKeys.Button.A))
                .whenPressed(setJunctionLow);
        moveMedium = (new GamepadButton(driverOp2, GamepadKeys.Button.B))
                .whenPressed(setJunctionMedium);
        moveHigh = (new GamepadButton(driverOp2, GamepadKeys.Button.Y))
                .whenPressed(setJunctionHigh);

        moveFirst = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_UP))
                .whenPressed(setFirst);
        moveSecond = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_LEFT))
                .whenPressed(setSecond);
        moveThird = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_RIGHT))
                .whenPressed(setThird);
        moveFourth = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(setFourth);

        register(drive, lift, claw, slide);
        drive.setDefaultCommand(robotCentricDrive);
        lift.setDefaultCommand(moveLiftPID);

    }



}