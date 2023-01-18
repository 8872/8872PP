package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.command.group.GrabAndLift;
import org.firstinspires.ftc.teamcode.command.group.LiftDown;
import org.firstinspires.ftc.teamcode.command.group.LiftUp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.SlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.util.Junction;


@Disabled
@Config
@Autonomous
public class TestAuto extends BaseOpMode {

    public static double initial_x_pos = 55.56;
    public static double initial_y_pos = -2;
    public static double initial_turn_angle = 110;
    public static double spline_x_pos = 51;
    public static double spline_y_pos = 5.5;
    public static double retrieve_y_pos = 27.33;
    public static double deposit_x_pos = 54.75;
    public static double deposit_y_pos = -3.33;
    public static double x_change = 0.3;
    public static double y_change = 0.3;

    private MotorEx dr4bLeftMotor, dr4bRightMotor;
    private SimpleServo claw, slide;
    private TouchSensor limitSwitch;

    private SampleMecanumDrive drive;
    private LiftSubsystem liftSub;
    private ClawSubsystem clawSub;
    private SlideSubsystem slideSub;



    int pickupPosition = -120;
    int coneCounter = 3;

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

    DRIVE_PHASE currentState = DRIVE_PHASE.IDLE;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        double storedSplineX = spline_x_pos;
        double storedDepositX = deposit_x_pos;
        double storedDepositY = deposit_y_pos;

        dr4bLeftMotor = new MotorEx(hardwareMap, "dr4bLeft");
        dr4bRightMotor = new MotorEx(hardwareMap, "dr4bRight");
        dr4bLeftMotor.resetEncoder();
        dr4bRightMotor.resetEncoder();

        claw = new SimpleServo(hardwareMap, "claw", 0, 120);
        slide = new SimpleServo(hardwareMap, "slide", 0, 120);

        limitSwitch = hardwareMap.get(TouchSensor.class, "touch");

        drive = new SampleMecanumDrive(hardwareMap);
        slideSub = new SlideSubsystem(slide);
        liftSub = new LiftSubsystem(dr4bLeftMotor,dr4bRightMotor,limitSwitch);
        clawSub = new ClawSubsystem(claw);

        drive.setPoseEstimate(startPose);

        ElapsedTime wait500 = new ElapsedTime();
        ElapsedTime wait300 = new ElapsedTime();

        telemetry.addData("initialized", true);
        telemetry.update();

        schedule(new GrabAndLift(liftSub, clawSub, -100));

        waitForStart();

        if (isStopRequested()) return;

        currentState = DRIVE_PHASE.WAIT_FOR_PRELOAD;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case WAIT_FOR_PRELOAD:
                    schedule(new LiftUp(liftSub, slideSub, Junction.HIGH));
//                    drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(startPose)
//                            .lineToLinearHeading(new Pose2d(initial_x_pos,initial_y_pos,Math.toRadians(initial_turn_angle)))
//                            .build());
                    currentState = DRIVE_PHASE.DEPOSIT;
                    break;
                case DEPOSIT:
                    if (!drive.isBusy()) {
                        currentState = DRIVE_PHASE.WAIT_FOR_DEPOSIT;
                        wait500.reset();
                    }
                    break;

                case WAIT_FOR_DEPOSIT:
                    if (wait500.seconds() >= 0.5) {
                        clawSub.release();
                        wait300.reset();
                        currentState = DRIVE_PHASE.MOVE_TO_RETRIEVE;
                    }
                    break;
                case MOVE_TO_RETRIEVE:
                    if(wait300.seconds() >= 0.3){
                        schedule(new LiftDown(liftSub, slideSub, clawSub));
                        if(coneCounter <= 0){
                            currentState = DRIVE_PHASE.PARK;
                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .turn(Math.toRadians(-45))
                                    .build());
                        }else{
                            currentState = DRIVE_PHASE.RETRIEVE;
//                            drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                    .splineTo(new Vector2d(spline_x_pos, spline_y_pos), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                                    .forward(Math.abs(retrieve_y_pos-spline_y_pos), SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                                    .build());
                            spline_x_pos += x_change;
                        }
                        coneCounter--;

                    }
                    break;

                case RETRIEVE:
                    if (!drive.isBusy()) {
                        schedule(new GrabAndLift(liftSub, clawSub, liftSub.getLeftEncoderValue()-100));
                        currentState = DRIVE_PHASE.WAIT_FOR_GRAB;
                        wait500.reset();
                    }
                    break;
                case WAIT_FOR_GRAB:
                    if(wait500.seconds()>=0.5){
                        schedule(new LiftUp(liftSub, slideSub, Junction.HIGH));
                        currentState = DRIVE_PHASE.DEPOSIT;
//                        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .back(Math.abs(spline_y_pos-deposit_y_pos),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                                .splineTo(new Vector2d(deposit_x_pos, deposit_y_pos), Math.toRadians(312.64),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                                .build());
                        deposit_x_pos += x_change;
                        deposit_y_pos += y_change;
                    }

                    break;
                case PARK:
                    if(!drive.isBusy()){
                        currentState = DRIVE_PHASE.IDLE;
                    }
                    break;
                case IDLE:
                    deposit_y_pos = storedDepositY;
                    deposit_x_pos = storedDepositX;
                    spline_x_pos = storedSplineX;
                    break;
            }

            drive.update();
            liftSub.updatePID();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("drive phase", currentState);
            telemetry.addData("slide", slideSub.getPosition());
            telemetry.addData("lift", liftSub.getTargetPosition());
            telemetry.addData("liftPos", liftSub.getLeftEncoderValue());
            telemetry.update();
        }
    }
}
