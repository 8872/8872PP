package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    public final double PIX_TO_DEGREES = 70.42/320;
    private JunctionWithArea pipeline;
    private final AnalogInput turretEnc;
    private final ServoEx turret;
    private boolean trackingMode;
    private TurretPIDF turretPIDF;
    public double currentPos;
    public double change;
    public double target;

    public enum Pose implements Position {
        RIGHT_FORWARD(0.9875),
        LEFT_FORWARD(0.135),
        RIGHT_BACK(0.67),
        LEFT_BACK(0.43),
        LEFT(0.29),
        RIGHT(0.832),
        ZERO(0.557),
        ONE_EIGHTY(0);

        private final double height;

        Pose(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    public TurretSys(ServoEx turret, AnalogInput turretEnc, TurretPIDF turretPIDF) {
        super(turret, maxVelocity, maxAcceleration);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                new TrapezoidProfile.State(Pose.ZERO.getHeight(), 0));
        currentTarget = Pose.ZERO.getHeight();
        this.turretEnc = turretEnc;
        this.turret = turret;
        this.turretPIDF = turretPIDF;
        trackingMode = false;
    }

    @Override
    public void periodic() {
        if (!trackingMode) {
            super.periodic();
        } else {
            double current = time.milliseconds();
            if (current >= 200){
                updateTarget();
                currentPos = getEncoderPosition();
                currentPos = currentPos > 355 ? 355 : currentPos;
                currentPos = currentPos < 0 ? 0 : currentPos;
                double factor = target < currentPos ? 5 : -5;
                turret.turnToAngle(Math.ceil(currentPos)+factor);
                Log.d("asd", "floored currentPos: "+Math.floor(currentPos));
                Log.d("asd", "turret angle: " + turret.getAngle());
                Log.d("asd", "target position: " + target);
                time.reset();
            }


//            change = turretPIDF.calculate(target - currentPos);

//            profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 2),
//                    new TrapezoidProfile.State(target, 0));
//            double factor = target < currentPos ? -1 : 1;
//            if(current >= 500) {
//                Log.d("asd", "target: " + target);
//                Log.d("asd", "currentPos: " + currentPos);
//                Log.d("asd", "turret position: " + turret.getAngle());
//                turret.turnToAngle(currentPos+factor);
//                time.reset();
//            }
        }
    }

    public void setPipeline(JunctionWithArea pipeline){
        this.pipeline = pipeline;
        if(this.pipeline == null)
            Log.println(Log.WARN, "pipeline null", "pipeline null");
    }

    public void updateTarget(){
        Rect rect = pipeline.getRect();
        if(rect == null)
            return;

        int junctionX = rect.x+rect.width/2;
        double turretPos = getEncoderPosition();
        target = (turretPos + ((junctionX-160)*PIX_TO_DEGREES));
    }

    public double getEncoderPosition(){
        double angle = (turretEnc.getVoltage() - 0.167) / 2.952 * 355;
        return -angle+355;
    }

    //TODO: make regular and follow mode transition nice
    public void stopTracking(){
        trackingMode = false;
    }

    public void startTracking(){
        time.reset();
        trackingMode = true;
    }


}
