package org.firstinspires.ftc.teamcode.roadrunner.drive.smantAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes.CycleTest;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

@Autonomous
public class autoCycle extends LinearOpMode {


    private MotorEx dr4bLeftMotor, dr4bRightMotor;
    private SimpleServo claw, slide;
    private TouchSensor limitSwitch;

    private SampleMecanumDrive drive;
    private ArmSubsystem arm;

    private Trajectory preload, retrieve, deposit, park;

    private double waitTime1;
    private ElapsedTime waitTimer1;

    int pickupPosition = -120;
    int coneCounter = 4;

    private enum DRIVE_PHASE {
        DEPOSIT,
        WAIT,
        RETRIEVE,
        PARK,
        IDLE
    }
    boolean waitingForExtend = false;
    boolean waitingForRetract = false;
    private double waitTime2;
    private ElapsedTime waitTimer2;

    boolean waitingForReopen = false;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));


    @Override
    public void runOpMode() throws InterruptedException {

        dr4bLeftMotor = new MotorEx(hardwareMap, "dr4bLeft");
        dr4bRightMotor = new MotorEx(hardwareMap, "dr4bRight");
        dr4bLeftMotor.resetEncoder();
        dr4bRightMotor.resetEncoder();

        claw = new SimpleServo(hardwareMap, "claw", 0, 120);
        slide = new SimpleServo(hardwareMap, "slide", 0, 120);

        limitSwitch = hardwareMap.get(TouchSensor.class, "touch");

        drive = new SampleMecanumDrive(hardwareMap);
        arm = new ArmSubsystem(claw, slide, dr4bLeftMotor, dr4bRightMotor, limitSwitch);

        drive.setPoseEstimate(startPose);


        Trajectory initalCone = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(54, -1.6, Math.toRadians(143.5)))

                .build();


        drive.followTrajectory(initalCone);
        arm.setJunction(ArmSubsystem.Junction.HIGH);
        arm.release();





    }


}