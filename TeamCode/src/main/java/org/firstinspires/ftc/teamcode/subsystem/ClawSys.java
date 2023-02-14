package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class ClawSys extends SubsystemBase {
    private final ServoEx claw;
    public static double grabPosition = 0.85;
    public static double releasePosition = 0;

    public ClawSys(ServoEx claw) {
        this.claw = claw;

    }

    public Command grab() {
        return new InstantCommand(() -> claw.setPosition(grabPosition), this);
    }

    public Command release() {
        return new InstantCommand(() -> claw.setPosition(releasePosition), this);
    }
}
