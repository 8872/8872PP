package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

@Config
@Autonomous
public class CycleTest extends LinearOpMode {

    public static double initial_x_pos = 55;
    public static double initial_y_pos = -4.5;
    public static double spline_x_pos = 52;
    public static double spline_y_pos = 5;
    public static double retrieve_x_pos = 52;
    public static double retrieve_y_pos = 25.5;
    public static double deposit_x_pos = 55;
    public static double deposit_y_pos = -4.5;

    private MotorEx dr4bLeftMotor, dr4bRightMotor;
    private SimpleServo claw, slide;
    private TouchSensor limitSwitch;

    private SampleMecanumDrive drive;
    private ArmSubsystem arm;

    private TrajectorySequence preload;
    private TrajectorySequenceBuilder preload_turn;
    private TrajectorySequence retrieve;
    private TrajectorySequence deposit;//, park, skuffedFix;

    private double waitTime1;
    private ElapsedTime waitTimer1;

    int pickupPosition = -130;
    int coneCounter = 5;

    private enum DRIVE_PHASE {
        INITIAL_GRAB,
        WAIT_FOR_PRELOAD,
        DEPOSIT,
        WAIT_FOR_DEPOSIT,
        MOVE_TO_RETRIEVE,
        RETRIEVE,
        WAIT_FOR_GRAB,
        PARK,
        IDLE
    }
    boolean waitingForExtend = false;
    boolean waitingForRetract = false;
    boolean waitingForLower = false;
    private double waitTime2;
    private ElapsedTime waitTimer2;
    private ElapsedTime waitTimerInitial;
    double waitTimeInitial;

    boolean waitingForReopen = false;

    DRIVE_PHASE currentState = DRIVE_PHASE.IDLE;
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


        /*
        preload = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(initial_x_pos, initial_y_pos))
                .turn(2.315)
                .build();

        retrieve = drive.trajectorySequenceBuilder(preload.end())
                .splineTo(new Vector2d(spline_x_pos, spline_y_pos), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(Math.abs(retrieve_y_pos-spline_y_pos), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        deposit = drive.trajectorySequenceBuilder(retrieve.end())
                .back(Math.abs(spline_y_pos-deposit_y_pos),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(deposit_x_pos, deposit_y_pos), Math.toRadians(312.64),SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
         */
        //TODO: make it park based on the sleeve
//        park = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(0)))
//                .build();

        waitTime1 = 0.5;
        waitTimer1 = new ElapsedTime();

        waitTime2 = 1;
        waitTimer2 = new ElapsedTime();

        waitTimeInitial = 1.0;
        waitTimerInitial = new ElapsedTime();

        ElapsedTime waitTimer3 = new ElapsedTime();
        double waitTime3 = 0.5;

        ElapsedTime waitTimerRetrieve = new ElapsedTime();
        double waitTimeRetrieve = 0.5;


        telemetry.addData("initialized", true);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;



        currentState = DRIVE_PHASE.WAIT_FOR_PRELOAD;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case WAIT_FOR_PRELOAD:
                    if(waitTimerInitial.seconds() <= waitTimeInitial && !drive.isBusy()){
                        arm.grab();
                        sleep(1000);
                        arm.setJunction(ArmSubsystem.Junction.HIGH);
                        waitingForExtend = true;
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(startPose)
                                .lineToConstantHeading(new Vector2d(initial_x_pos, initial_y_pos))
                                .turn(2.315)
                                .build());
                        currentState = DRIVE_PHASE.DEPOSIT;
                    }
                    break;
                case DEPOSIT:
                    if (!drive.isBusy()) {
                        currentState = DRIVE_PHASE.WAIT_FOR_DEPOSIT;
                        waitTimer1.reset();
                    }
                    break;

                case WAIT_FOR_DEPOSIT:
                    if (waitTimer1.seconds() >= waitTime1) {
                        arm.release();
                        waitTimerRetrieve.reset();
                        currentState = DRIVE_PHASE.MOVE_TO_RETRIEVE;
                    }
                    break;
                case MOVE_TO_RETRIEVE:
                    if(waitTimerRetrieve.seconds() >= waitTimeRetrieve){
                        arm.grab();
                        arm.slideIn();
                        waitingForReopen = true;
                        waitTimer2.reset();
                        waitingForLower = true;
                        waitTimer3.reset();
                        if(coneCounter <= 0){
                            currentState = DRIVE_PHASE.PARK;
                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .turn(Math.toRadians(-312.64))
                                    .build());
                        }else{
                            currentState = DRIVE_PHASE.RETRIEVE;
                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .splineTo(new Vector2d(spline_x_pos, spline_y_pos), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .forward(Math.abs(retrieve_y_pos-spline_y_pos), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build());
                        }
                        coneCounter--;

                    }
                    break;

                case RETRIEVE:
                    if (!drive.isBusy()) {
                        arm.grab();
                        arm.setJunction(ArmSubsystem.Junction.HIGH);
                        waitTimer2.reset();
                        currentState = DRIVE_PHASE.WAIT_FOR_GRAB;
                    }
                    break;
                case WAIT_FOR_GRAB:
                    if(waitTimer2.seconds()>=waitTime2){
                        waitingForExtend = true;
                        currentState = DRIVE_PHASE.DEPOSIT;
                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .back(Math.abs(spline_y_pos-deposit_y_pos),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .splineTo(new Vector2d(deposit_x_pos, deposit_y_pos), Math.toRadians(312.64),SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build());
                    }
                    break;
                case PARK:
                    if(!drive.isBusy()){
                        currentState = DRIVE_PHASE.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            if(waitingForExtend && arm.getLeftEncoderValue()<-350){
                waitingForExtend = false;
                arm.slideOut();
            }
            if(waitingForRetract && waitTimer2.seconds() >= waitTime2){
                arm.slideIn();
                arm.grab();
                waitingForRetract = false;
                waitingForReopen = true;
                waitTimer1.reset();
            }
            if(waitingForReopen && waitTimer2.seconds() >= waitTime2){
                arm.release();
                waitingForReopen = false;
            }
            if(waitingForLower && waitTimer3.seconds() >= waitTime3){
                arm.setJunction(pickupPosition);
                pickupPosition += 28;
                waitingForLower = false;
            }


            drive.update();
            arm.loopPID();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("drive phase", currentState);
            telemetry.update();
        }
    }
}
