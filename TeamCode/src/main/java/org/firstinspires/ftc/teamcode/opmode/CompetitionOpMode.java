package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.claw.MoveClaw;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.drive.DefaultFieldCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.DefaultRobotCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.SlowMode;
import org.firstinspires.ftc.teamcode.command.lift.*;
import org.firstinspires.ftc.teamcode.command.slide.MoveSlide;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

@TeleOp(name = "final opmode ig pls work ;-;")
public class CompetitionOpMode extends BaseOpMode {
    private GamepadEx driverOp1;
    private GamepadEx driverOp2;

    private DefaultFieldCentricDrive fieldCentricDrive;
    private DefaultRobotCentricDrive robotCentricDrive;
    private SlowMode slowMode;

    private MoveSlide moveSlide;
    private MoveClaw moveClaw;
    private ManualLift manualLift;
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


        grabCone = new GrabCone(arm);
        releaseCone = new ReleaseCone(arm);
        clawStuff = (new GamepadButton(driverOp2, GamepadKeys.Button.LEFT_BUMPER))
                .toggleWhenPressed(grabCone, releaseCone);

        slideIn = new SlideIn(arm);
        slideOut = new SlideOut(arm);
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

        moveLiftPID = new MoveLiftPID(arm, () -> driverOp2.getRightY());

        setJunctionNone = new SetJunction(arm, ArmSubsystem.Junction.NONE);
        setJunctionGround = new SetJunction(arm, ArmSubsystem.Junction.GROUND);
        setJunctionLow = new SetJunction(arm, ArmSubsystem.Junction.LOW);
        setJunctionMedium = new SetJunction(arm, ArmSubsystem.Junction.MEDIUM);
        setJunctionHigh = new SetJunction(arm, ArmSubsystem.Junction.HIGH);

        moveNone = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(setJunctionNone);
        moveGround = (new GamepadButton(driverOp2, GamepadKeys.Button.X))
                .whenPressed(setJunctionGround);
        moveLow = (new GamepadButton(driverOp2, GamepadKeys.Button.A))
                .whenPressed(setJunctionLow);
        moveMedium = (new GamepadButton(driverOp2, GamepadKeys.Button.B))
                .whenPressed(setJunctionMedium);
        moveHigh = (new GamepadButton(driverOp2, GamepadKeys.Button.Y))
                .whenPressed(setJunctionHigh);

        register(drive, arm);
        drive.setDefaultCommand(robotCentricDrive);
        arm.setDefaultCommand(moveLiftPID);

    }



}