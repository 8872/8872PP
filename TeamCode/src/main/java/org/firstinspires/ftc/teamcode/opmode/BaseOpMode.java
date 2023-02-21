package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.TriggerGamepadEx;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.firstinspires.ftc.teamcode.vision.util.TurretPIDF;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class BaseOpMode extends CommandOpMode {
    protected MotorEx fL, fR, bL, bR, dr4bLeftMotor, dr4bRightMotor;
    protected SimpleServo clawServo, turretServo, armServo, flipperServo;
    protected DriveSys drive;
    protected LiftSys lift;
    protected ClawSys claw;
    protected TurretSys turret;
    protected ArmSys arm;
    protected FlipperSys flipper;
    protected RevIMU imu;
    protected TouchSensor limitSwitch;

    protected AnalogInput turretEnc;

    protected GamepadEx gamepadEx1;
    protected GamepadEx gamepadEx2;
    protected TriggerGamepadEx triggerGamepadEx1;
    protected TriggerGamepadEx triggerGamepadEx2;

    protected SampleMecanumDrive rrDrive;

    protected JunctionWithArea pipeline;
    protected OpenCvCamera camera;
    protected TurretPIDF turretPIDF;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);

        initHardware();
        setUpHardwareDevices();

        imu = new RevIMU(hardwareMap);
        imu.init();

        drive = new DriveSys(fL, fR, bL, bR, imu);
        lift = new LiftSys(dr4bLeftMotor, dr4bRightMotor, limitSwitch);
        lift.goTo(Height.NONE);

        claw = new ClawSys(clawServo);
        turret = new TurretSys(turretServo, turretEnc, turretPIDF);
        //turret = new TurretSys(turretServo, () -> gamepadEx2.getButton(GamepadKeys.Button.DPAD_DOWN), turretEnc);
        arm = new ArmSys(armServo);
        flipper = new FlipperSys(flipperServo);

        rrDrive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        pipeline = new JunctionWithArea();
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setPipeline(pipeline);

        turretPIDF = new TurretPIDF(() -> camera.getFps());

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tad("Mode", "Done initializing");
        telemetry.update();
    }

    protected void initHardware() {
        fL = new MotorEx(hardwareMap, "leftFront");
        fR = new MotorEx(hardwareMap, "rightFront");
        bL = new MotorEx(hardwareMap, "leftBack");
        bR = new MotorEx(hardwareMap, "rightBack");
        dr4bLeftMotor = new MotorEx(hardwareMap, "dr4bLeft");
        dr4bRightMotor = new MotorEx(hardwareMap, "dr4bRight");
        // what the proper min and max?
        clawServo = new SimpleServo(hardwareMap, "claw", 0, 120);
        // TODO change to 355 when switch to axon
        turretServo = new SimpleServo(hardwareMap, "turret", 0, 300);
        armServo = new SimpleServo(hardwareMap, "arm", 0, 355);
        flipperServo = new SimpleServo(hardwareMap, "flipper", 0, 355);

        turretEnc = hardwareMap.get(AnalogInput.class, "turretEnc");

        limitSwitch = hardwareMap.get(TouchSensor.class, "touch");
        dr4bLeftMotor.resetEncoder();
        dr4bRightMotor.resetEncoder();
    }


    @Override
    public void run() {
        super.run();
        tad("leftFront Power", round(fL.motor.getPower()));
        tad("leftBack Power", round(bL.motor.getPower()));
        tad("rightFront Power", round(fR.motor.getPower()));
        tad("rightBack Power", round(bR.motor.getPower()));
        tad("dr4bLeftMotor Power", round(dr4bLeftMotor.motor.getPower()));
        tad("dr4bRightMotor Power", round(dr4bRightMotor.motor.getPower()));
        tad("dr4bLeftMotor Position", dr4bLeftMotor.getCurrentPosition());
        tad("dr4bRightMotor Position", dr4bRightMotor.getCurrentPosition());

        tad("Lift Target Position", lift.getCurrentGoal());

        tad("Drive Heading PID Output", drive.getOutput());
        tad("Drive Heading Target", drive.getTarget());
        tad("Claw Position", clawServo.getPosition());
        tad("Turret Position", turretServo.getPosition());
        tad("Arm Position", armServo.getPosition());

        tad("IMU Heading", imu.getAbsoluteHeading());

        tad("Limit Switch Pressed", limitSwitch.isPressed());

        double turretPosition = turretEnc.getVoltage() / 3.3 * 360;
        tad("turret position", turretPosition);
        telemetry.update();
    }

    protected void setUpHardwareDevices() {
        fL.setInverted(true);
        bL.setInverted(true);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        dr4bLeftMotor.setRunMode(Motor.RunMode.RawPower);
        dr4bRightMotor.setRunMode(Motor.RunMode.RawPower);
        dr4bLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dr4bRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }


    private static double round(double value, @SuppressWarnings("SameParameterValue") int places) {
        if (places < 0) throw new IllegalArgumentException();

        return new BigDecimal(Double.toString(value)).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    private static double round(double value) {
        return round(value, 4);
    }

    // gamepad button 1 = gb1
    protected GamepadButton gb1(GamepadKeys.Button button) {
        return gamepadEx1.getGamepadButton(button);
    }

    // gamepad button 2 = gb2
    protected GamepadButton gb2(GamepadKeys.Button button) {
        return gamepadEx2.getGamepadButton(button);
    }

    protected GamepadTrigger gb1(GamepadKeys.Trigger trigger) {
        return triggerGamepadEx1.getGamepadTrigger(trigger);
    }

    protected GamepadTrigger gb2(GamepadKeys.Trigger trigger) {
        return triggerGamepadEx2.getGamepadTrigger(trigger);
    }

    // telemetry add data = tad
    protected void tad(String caption, Object value) {
        telemetry.addData(caption, value);
    }
}
