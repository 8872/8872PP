package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class TestServoOpmode extends OpMode {
    SimpleServo flipper;
    public static double inPos = 1;
    public static double outPos = 0.61;
    @Override
    public void init(){
        flipper = new SimpleServo(hardwareMap, "flipper", 0, 355);
    }
    @Override
    public void loop(){
        if(gamepad1.x){
            flipper.setPosition(inPos);
        }
        if(gamepad1.b){
            flipper.setPosition(outPos);
        }
    }


}
