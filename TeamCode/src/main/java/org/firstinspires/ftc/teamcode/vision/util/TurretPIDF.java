package org.firstinspires.ftc.teamcode.vision.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.MedianFilter;

import java.util.function.DoubleSupplier;

@Config
public class TurretPIDF {

    public static double Kp = 0.2;//0.15;
    public static double Ki = 0;
    public static double Kd = 3;
    public static double Kf = 4;
    public static double MAX_VELOCITY = 100;
    private static double totalError = 0;
    private static double previousError = 0;


    public static double calculateDSq(double error, ElapsedTime time){
        double errorVal_v;
        if (time.milliseconds() > 1E-6) {
            errorVal_v = (error) / time.milliseconds();
        } else {
            errorVal_v = 0;
        }

        double targetVel = error * Math.abs(error) * Kp + Kf * error / Math.abs(error) + Kd * errorVal_v;
        //if(targetVel > MAX_VELOCITY) targetVel = MAX_VELOCITY;
        return targetVel*time.seconds();
    }


    public static double calculateP(double error, ElapsedTime time, double Kp){
        double targetVel = error * Kp + Kf * error / Math.abs(error);
        if(targetVel > MAX_VELOCITY) targetVel = MAX_VELOCITY;
        return targetVel*time.seconds();
    }
    public static double calculateD(double error, ElapsedTime time){
        double errorVal_v;
        if (time.milliseconds() > 1E-6) {
            errorVal_v = (error) / time.milliseconds();
        } else {
            errorVal_v = 0;
        }

        double targetVel = error * Kp + Kf + Kd * errorVal_v;
        if(targetVel > MAX_VELOCITY) targetVel = MAX_VELOCITY;
        return targetVel*time.seconds();
    }
    public static double calculateI(double error, ElapsedTime time){
//        if (crossOverDetected(error,previousError)) totalError = 0;
        totalError += ((error + previousError) / 2) * time.milliseconds();

        double errorVal_v;
        if (time.milliseconds() > 1E-6) {
            errorVal_v = (error) / time.milliseconds();
        } else {
            errorVal_v = 0;
        }

        double targetVel = error * Kp + Kf + Kd * errorVal_v + Ki*totalError;
        if(targetVel > MAX_VELOCITY) targetVel = MAX_VELOCITY;
        previousError = error;

        return targetVel*time.seconds();
    }
    private static boolean crossOverDetected(double error, double prev) {
        if (error > 0 && prev < 0) return true;
        return error < 0 && prev > 0;
    }

}