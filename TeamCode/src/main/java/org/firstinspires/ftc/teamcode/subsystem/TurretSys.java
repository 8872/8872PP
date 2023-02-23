package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.util.LinearFilter;
import org.firstinspires.ftc.teamcode.util.MinFilter;
import org.firstinspires.ftc.teamcode.util.Position;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.firstinspires.ftc.teamcode.vision.util.TurretPIDF;
import org.opencv.core.Rect;

import java.util.function.BooleanSupplier;

@Config
public final class TurretSys extends ProfiledServoSubsystem {
    public static double maxVelocity = 3;
    public static double maxAcceleration = 3;
    private JunctionWithArea pipeline;
    private final AnalogInput turretEnc;
    private final ServoEx turret;
    public static double pix_to_degree = 0.192;
    private boolean trackingMode;
    MinFilter minFilter = new MinFilter(50);
    LinearFilter lowpass = LinearFilter.singlePoleIIR(0.1, 0.02);
    public double turretPosition = 0;
    public double targetPos;
    public double change;
    public double target;
    Rect rect;


    public enum Pose implements Position {
        RIGHT_FORWARD(0.0125),
        LEFT_FORWARD(0.865),
        RIGHT_BACK(0.33),
        LEFT_BACK(0.57),
        LEFT(0.71),
        RIGHT(0.168),
        ZERO(0.43),
        ONE_EIGHTY(1);

        private final double height;

        Pose(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    public TurretSys(ServoEx turret, AnalogInput turretEnc) {
        super(turret, maxVelocity, maxAcceleration);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                new TrapezoidProfile.State(Pose.ZERO.getHeight(), 0));
        currentTarget = Pose.ZERO.getHeight();
        this.turretEnc = turretEnc;
        this.turret = turret;
        trackingMode = false;
    }

    @Override
    public void periodic() {
        updatePosition();
        if (!trackingMode) {
            super.periodic();
        } else {
            if((rect = pipeline.getRect()) != null){
                updateTarget(rect);
                double error = target-turretPosition;
                change = TurretPIDF.calculateD(error, time);
                time.reset();
                if(targetPos+change > 0 && targetPos+change < 350)
                    targetPos += change;
                turret.turnToAngle(targetPos);

//                updateTarget(rect);
//                double error = turretPosition-target;
//                change = TurretPIDF.calculateP(error, time);
//                time.reset();
//                targetPos = target-turretPosition < 0 ? targetPos - change : targetPos + change;
//                turret.turnToAngle(targetPos);
            }
        }
    }

    public void setPipeline(JunctionWithArea pipeline){
        this.pipeline = pipeline;
        if(this.pipeline == null)
            Log.println(Log.WARN, "pipeline null", "pipeline null");
    }

    public void updatePosition(){
        double angle = (turretEnc.getVoltage() - 0.189) / 2.957 * 355;
        turretPosition = angle;//minFilter.calculate(lowpass.calculate(angle));
    }

    public void updateTarget(Rect rect){
        double junctionX = rect.x + (double) rect.width / 2;
        target = (turretPosition + ((junctionX-160)*pix_to_degree));
    }

    public void startTracking(){
        trackingMode = true;
        targetPos = turretPosition;
        time.reset();
    }

    public void stopTracking(){
        trackingMode = false;
    }

}
