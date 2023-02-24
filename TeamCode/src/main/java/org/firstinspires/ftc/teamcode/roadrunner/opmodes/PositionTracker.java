package org.firstinspires.ftc.teamcode.roadrunner.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.command.group.MediumSequence;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp
public class PositionTracker extends LinearOpMode {
    private AnalogInput turretEnc;
    private SimpleServo turretServo;
    private double setPosition = 0.43;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        turretEnc = hardwareMap.get(AnalogInput.class, "turretEnc");
        turretServo = new SimpleServo(hardwareMap, "turret", 0, 355);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(36, -62, Math.toRadians(90)));
        turretServo.setPosition(setPosition);

        waitForStart();


        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepad1.x){
                turretServo.setPosition(turretServo.getPosition()+0.001);
            }
            else if(gamepad1.b){
                turretServo.setPosition(turretServo.getPosition()-0.001);
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("turret", (turretEnc.getVoltage() - 0.189) / 2.957 * 355);
            telemetry.update();
        }
    }
}
