package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import org.firstinspires.ftc.teamcode.opmode.MainOpMode;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.vision.JunctionDetection;
import org.opencv.core.Point;

public class AdjustAngle extends CommandBase {
    public final DriveSubsystem drive;
    public final JunctionDetection pipeline;
    public final RevIMU imu;
    public AdjustAngle(DriveSubsystem drive, JunctionDetection pipeline, RevIMU imu){
        this.drive = drive;
        this.pipeline = pipeline;
        this.imu = imu;
        addRequirements(drive);
    }
    @Override
    public void initialize() {
        Point center;
        while((center = pipeline.uniqueCenter()) == null) continue;
        double error = (center.x-640)/MainOpMode.PIX_TO_DEGREE;
        drive.setHeading(imu.getHeading()-error);
    }
    @Override
    public void execute(){
        drive.updatePID();
    }
    @Override
    public boolean isFinished(){
        return drive.atDesiredAngle();
    }
}
