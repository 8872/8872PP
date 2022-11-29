package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.claw.MoveClaw;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.command.drive.DefaultFieldCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.DefaultRobotCentricDrive;
import org.firstinspires.ftc.teamcode.command.lift.*;
import org.firstinspires.ftc.teamcode.command.slide.MoveSlide;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.slide.SlideOut;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

@TeleOp(name = "final opmode ig")
public class TeleOpMode extends BaseOpMode {
    private GamepadEx driverOp1;
    private GamepadEx driverOp2;

    private DefaultFieldCentricDrive fieldCentricDrive;
    private DefaultRobotCentricDrive robotCentricDrive;
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

        changeCenter = (new GamepadButton(driverOp1, GamepadKeys.Button.LEFT_STICK_BUTTON)).
                toggleWhenPressed(fieldCentricDrive, robotCentricDrive);

        grabCone = new GrabCone(arm);
        releaseCone = new ReleaseCone(arm);
        clawStuff = (new GamepadButton(driverOp2, GamepadKeys.Button.B))
                .toggleWhenPressed(grabCone, releaseCone);

        slideIn = new SlideIn(arm);
        slideOut = new SlideOut(arm);
        slideStuff = (new GamepadButton(driverOp2, GamepadKeys.Button.Y))
                .toggleWhenPressed(slideOut, slideIn);

//        moveSlide = new MoveSlide(arm, () -> driverOp2.getLeftY());
//        moveClaw = new MoveClaw(arm, () -> driverOp2.getRightX());


        resetEncoders = (new GamepadButton(driverOp2, GamepadKeys.Button.A)).whenPressed(
                new InstantCommand(() -> {
                    dr4bLeftMotor.resetEncoder();
                    dr4bRightMotor.resetEncoder();
                }, arm));


//        moveConeHigh = new MoveConeHigh(arm);
//        moveHigh = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_UP))
//                .whenPressed(moveConeHigh);
//
//        moveConeMedium = new MoveConeMedium(arm);
//        moveHigh = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_RIGHT))
//                .whenPressed(moveConeMedium);
//
//        moveConeLow = new MoveConeLow(arm);
//        moveLow = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_LEFT))
//                .whenPressed(moveConeLow);

        moveLiftPID = new MoveLiftPID(arm);

        setJunctionNone = new SetJunction(arm, ArmSubsystem.Junction.NONE);
        setJunctionGround = new SetJunction(arm, ArmSubsystem.Junction.GROUND);
        setJunctionLow = new SetJunction(arm, ArmSubsystem.Junction.LOW);
        setJunctionMedium = new SetJunction(arm, ArmSubsystem.Junction.MEDIUM);
        setJunctionHigh = new SetJunction(arm, ArmSubsystem.Junction.HIGH);

        moveNone = (new GamepadButton(driverOp2, GamepadKeys.Button.X))
                .whenPressed(setJunctionGround);
        moveGround = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_DOWN))
                .whenPressed(setJunctionGround);
        moveLow = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_LEFT))
                .whenPressed(setJunctionLow);
        moveMedium = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_RIGHT))
                .whenPressed(setJunctionMedium);
        moveHigh = (new GamepadButton(driverOp2, GamepadKeys.Button.DPAD_UP))
                .whenPressed(setJunctionHigh);

        schedule(new RunCommand(() -> {
            if(limitSwitch.isPressed()){
                dr4bLeftMotor.resetEncoder();
                dr4bRightMotor.resetEncoder();
            }
        }, arm));




        register(drive, arm);
        drive.setDefaultCommand(robotCentricDrive);
        arm.setDefaultCommand(moveLiftPID);

    }



}
//mid:  848, 847, 859, 893, 853
//high: 2100, 2131, 2223, 1919, 1827, 1873, 2138, 1637: 1981
//low: 717, 716, 745, 707, 760