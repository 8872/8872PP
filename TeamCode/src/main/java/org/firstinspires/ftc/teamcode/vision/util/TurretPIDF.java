package org.firstinspires.ftc.teamcode.vision.util;

import java.util.function.DoubleSupplier;

public class TurretPIDF {

    public static final double MAX_ERROR = 0.11;

    public static double Kp = 1;//0.15;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.01;

    private DoubleSupplier fps;
    private boolean fpsGiven = true;

    public TurretPIDF(){
        fpsGiven = false;
    }

    public TurretPIDF(DoubleSupplier fps){
        this.fps = fps;
    }

    public double calculate(double error){
        double targetVel = ((error / MAX_ERROR) * Kp)*320;
        return fpsGiven ? targetVel / fps.getAsDouble() : targetVel / 30;
    }




}
