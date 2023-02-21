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
            updateTarget();
            turret.setPosition(turretPIDF.calculate(getEncoderPosition()-currentTarget));
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
        double turretPos = turretEnc.getVoltage() / 3.3 * 360;
        double newPos = -(turretPos + ((junctionX-160)*PIX_TO_DEGREES));
        currentTarget = ((newPos+20)/320+1);
    }

    public double getEncoderPosition(){
        if(turretEnc == null) {
            Log.println(Log.WARN, "turretEnc null", "turretEnc null");
            return 0;
        }
        double turretPos = turretEnc.getVoltage() / 3.3 * 360;
        return (-turretPos+20)/320+1;
    }

    public void stopTracking(){
        trackingMode = false;
    }

    public void startTracking(){
        trackingMode = true;
    }

}
