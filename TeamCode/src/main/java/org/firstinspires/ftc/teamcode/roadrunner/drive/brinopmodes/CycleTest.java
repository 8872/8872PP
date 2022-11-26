package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class CycleTest extends LinearOpMode {

    private MotorEx dr4bLeftMotor, dr4bRightMotor;
    private SimpleServo claw, slide;
    private TouchSensor limitSwitch;

    private SampleMecanumDrive drive;
    private ArmSubsystem arm;

    private Trajectory preload, retrieve, deposit, park;

    private double waitTime1;
    private ElapsedTime waitTimer1;

    int pickupPosition = -200;
    int coneCounter = 3;

    private enum DRIVE_PHASE {
        DEPOSIT,
        WAIT,
        RETRIEVE,
        PARK,
        IDLE
    }
    boolean waitingForExtend = false;

    DRIVE_PHASE currentState = DRIVE_PHASE.IDLE;
    Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(90));

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

        preload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-31, -4, Math.toRadians(30)))
                .build();

        retrieve = drive.trajectoryBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(-68, -12, Math.toRadians(0)))
                .build();

        deposit = drive.trajectoryBuilder(retrieve.end())
                .lineToLinearHeading(new Pose2d(-31, -4, Math.toRadians(30)))
                .build();

        //TODO: make it park based on the sleeve
        park = drive.trajectoryBuilder(preload.end())
                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        waitTime1 = 1.0;
        waitTimer1 = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        arm.setJunction(ArmSubsystem.Junction.HIGH);
        waitingForExtend = true;
        waitTimer1.reset();
        currentState = DRIVE_PHASE.DEPOSIT;
        drive.followTrajectoryAsync(preload);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case DEPOSIT:
                    if (!drive.isBusy()) {
                        currentState = DRIVE_PHASE.WAIT;
                        waitTimer1.reset();
                    }
                    break;

                case WAIT:
                    if (waitTimer1.seconds() >= waitTime1) {
                        arm.release();
                        arm.slideIn();
                        arm.setJunction(pickupPosition);
                        coneCounter--;
                        pickupPosition -= 50;
                        if(coneCounter<0){
                            currentState = DRIVE_PHASE.PARK;
                            drive.followTrajectoryAsync(park);
                        }else{
                            currentState = DRIVE_PHASE.RETRIEVE;
                            drive.followTrajectoryAsync(retrieve);
                        }
                    }
                    break;

                case RETRIEVE:
                    if (!drive.isBusy()) {
                        arm.grab();
                        arm.setJunction(ArmSubsystem.Junction.HIGH);
                        waitingForExtend = true;
                        currentState = DRIVE_PHASE.DEPOSIT;
                        drive.followTrajectoryAsync(deposit);
                    }
                    break;

                case PARK:
                    if(!drive.isBusy()){
                        currentState = DRIVE_PHASE.IDLE;
                    }

                case IDLE:
                    break;
            }
            if(waitingForExtend && arm.getLeftEncoderValue()<-250){
                waitingForExtend = false;
                arm.slideOut();
            }

            drive.update();
            arm.loopPID();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
