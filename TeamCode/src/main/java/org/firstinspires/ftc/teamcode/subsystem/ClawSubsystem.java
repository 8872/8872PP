package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class ClawSubsystem extends SubsystemBase {
    private final ServoEx claw;
    public static double grabPosition = 45;
    public static double releasePosition = 35;

    public ClawSubsystem(ServoEx claw) {
        this.claw = claw;

    }

    // grab cone
    public void grab() {
        claw.turnToAngle(grabPosition);
    }

    // release cone
    public void release() {
        claw.turnToAngle(releasePosition);
    }
}
