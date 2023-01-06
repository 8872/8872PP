package org.firstinspires.ftc.teamcode.roadrunner.drive.brinopmodes.autoCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import java.util.function.BooleanSupplier;

public class TrajectoryCommand extends CommandBase {
    private final Runnable trajectory;
    private final BooleanSupplier finished;

    public TrajectoryCommand(Runnable trajectory, BooleanSupplier finished, Subsystem... requirements){
        this.trajectory = trajectory;
        this.finished = finished;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        trajectory.run();
    }

    @Override
    public boolean isFinished() {
        return finished.getAsBoolean();
    }
}
