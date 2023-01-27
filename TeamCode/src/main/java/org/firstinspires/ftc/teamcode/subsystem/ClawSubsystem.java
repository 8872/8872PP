package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

@Config
public class ClawSubsystem extends SubsystemBase {
    private final ServoEx claw;
    public static double grabPosition = 35;
    public static double releasePosition = 45;

    public ClawSubsystem(ServoEx claw) {
        this.claw = claw;

    }

    public Command runGrabCommand() {
        return new InstantCommand(() -> claw.turnToAngle(grabPosition), this);
    }

    public Command runReleaseCommand() {
        return new InstantCommand(() -> claw.turnToAngle(releasePosition), this);
    }
}
