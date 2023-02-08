package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class ClawSys extends SubsystemBase {
    private final ServoEx claw;
    public static double grabPosition = 35;
    public static double releasePosition = 45;

    public ClawSys(ServoEx claw) {
        this.claw = claw;

    }

    public Command grab() {
        return new InstantCommand(() -> claw.turnToAngle(grabPosition), this);
    }

    public Command release() {
        return new InstantCommand(() -> claw.turnToAngle(releasePosition), this);
    }
}
