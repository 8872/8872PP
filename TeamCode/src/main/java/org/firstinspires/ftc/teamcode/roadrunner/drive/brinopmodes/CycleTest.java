package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;

@Disabled
@Autonomous
public class CycleTest extends LinearOpMode {

    private MotorEx dr4bLeftMotor, dr4bRightMotor;
    private SimpleServo clawServo, slideServo;
    private TouchSensor limitSwitch;

    private SampleMecanumDrive drive;
    private LiftSubsystem lift;
    private ClawSubsystem claw;
    private SlideSubsystem slide;


    private Trajectory preload, retrieve, deposit, park;

    private double waitTime1;
    private ElapsedTime waitTimer1;

    int pickupPosition = -140;
    int coneCounter = 4;

    private enum DRIVE_PHASE {
        WAIT_FOR_PRELOAD,
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

    DRIVE_PHASE currentState = DRIVE_PHASE.IDLE;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        dr4bLeftMotor = new MotorEx(hardwareMap, "dr4bLeft");
        dr4bRightMotor = new MotorEx(hardwareMap, "dr4bRight");
        dr4bLeftMotor.resetEncoder();
        dr4bRightMotor.resetEncoder();

        clawServo = new SimpleServo(hardwareMap, "claw", 0, 120);
        slideServo = new SimpleServo(hardwareMap, "slide", 0, 120);

        limitSwitch = hardwareMap.get(TouchSensor.class, "touch");

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new LiftSubsystem(dr4bLeftMotor, dr4bRightMotor, limitSwitch);
        claw = new ClawSubsystem(clawServo);
        slide = new SlideSubsystem(slideServo);


        drive.setPoseEstimate(startPose);
        preload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(56, 0.4, 2.18))
                .build();

        retrieve = drive.trajectoryBuilder(preload.end())
                .lineToSplineHeading(new Pose2d(50.1, 25.8, 1.66), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposit = drive.trajectoryBuilder(retrieve.end())
                .lineToLinearHeading(new Pose2d(56, 0.4, 2.18), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //TODO: make it park based on the sleeve
        park = drive.trajectoryBuilder(preload.end())
                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)))
                .build();

        waitTime1 = 1.0;
        waitTimer1 = new ElapsedTime();

        waitTime2 = 1;
        waitTimer2 = new ElapsedTime();

        double waitTimeInitial = 1;
        ElapsedTime waitTimerInitial = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        claw.grab();

        lift.setJunction(Junction.HIGH);
        waitingForExtend = true;
        waitTimer1.reset();
        currentState = DRIVE_PHASE.WAIT_FOR_PRELOAD;
        claw.grab();
        waitTimerInitial.reset();
        //drive.followTrajectoryAsync(preload);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case WAIT_FOR_PRELOAD:
                    claw.grab();
                    if(waitTimerInitial.seconds() <= waitTimeInitial){
                        drive.followTrajectoryAsync(preload);
                        currentState = DRIVE_PHASE.DEPOSIT;
                    }
                case DEPOSIT:
                    if (!drive.isBusy()) {
                        currentState = DRIVE_PHASE.WAIT;
                        waitTimer1.reset();

                    }
                    break;

                case WAIT:
                    if (waitTimer1.seconds() >= waitTime1) {
                        claw.release();
                        waitingForRetract = true;
                        waitTimer2.reset();
                        lift.setJunction(pickupPosition);
                        coneCounter--;
                        pickupPosition += 30;
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
                        claw.grab();
                        lift.setJunction(Junction.HIGH);
                        sleep(500);
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
            if(waitingForExtend && lift.getLeftEncoderValue()<-350){
                waitingForExtend = false;
                slide.out();
            }
            if(waitingForRetract && waitTimer2.seconds() >= waitTime2){
                slide.in();
                claw.grab();
                waitingForRetract = false;
                waitingForReopen = true;
                waitTimer1.reset();
            }
            if(waitingForReopen && waitTimer1.seconds() >= waitTime1){
                claw.release();
                waitingForReopen = false;
            }


            drive.update();
            lift.updatePID();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
