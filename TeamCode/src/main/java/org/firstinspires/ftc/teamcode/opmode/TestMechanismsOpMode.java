package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.claw.MoveClaw;
import org.firstinspires.ftc.teamcode.command.drive.DefaultFieldCentricDrive;
import org.firstinspires.ftc.teamcode.command.drive.DefaultRobotCentricDrive;
import org.firstinspires.ftc.teamcode.command.lift.MoveLift;
import org.firstinspires.ftc.teamcode.command.slide.MoveSlide;

//@Disabled // remove later
@TeleOp(name = "Test Mechanisms")
public class TestMechanismsOpMode extends BaseOpMode {
    private GamepadEx driverOp1;
    private GamepadEx driverOp2;

    private DefaultFieldCentricDrive fieldCentricDrive;
    private DefaultRobotCentricDrive robotCentricDrive;
    private MoveSlide moveSlide;
    private MoveClaw moveClaw;
    private MoveLift moveLift;

    private Button changeCenter;

    @Override
    public void initialize() {
        super.initialize();

        driverOp1 = new GamepadEx(gamepad1);
        driverOp2 = new GamepadEx(gamepad2);

        fieldCentricDrive = new DefaultFieldCentricDrive(drive, () -> driverOp1.getLeftX(),
                () -> driverOp1.getLeftY(), () -> driverOp1.getRightX(), () -> imu.getHeading());
        robotCentricDrive = new DefaultRobotCentricDrive(drive, () -> driverOp1.getLeftX(),
                () -> driverOp1.getLeftY(), () -> driverOp1.getRightX());

        changeCenter = (new GamepadButton(driverOp1, GamepadKeys.Button.A)).
                toggleWhenPressed(fieldCentricDrive, robotCentricDrive);

        moveSlide = new MoveSlide(arm, () -> driverOp2.getLeftY());
        moveClaw = new MoveClaw(arm, () -> driverOp2.getRightX());
        moveLift = new MoveLift(arm, () -> driverOp2.getRightY());





        register(drive, arm);
        drive.setDefaultCommand(robotCentricDrive);
    }

}
