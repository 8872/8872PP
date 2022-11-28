package org.firstinspires.ftc.teamcode.command.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.command.claw.GrabCone;
import org.firstinspires.ftc.teamcode.command.lift.*;
import org.firstinspires.ftc.teamcode.command.slide.DelayedSlideIn;
import org.firstinspires.ftc.teamcode.command.slide.SlideIn;
import org.firstinspires.ftc.teamcode.command.claw.ReleaseCone;
import org.firstinspires.ftc.teamcode.subsystem.ArmSubsystem;

public class ResetGround extends SequentialCommandGroup {
    public ResetGround(ArmSubsystem arm){

        addCommands(
                new GrabCone(arm),
                new MoveAbovePain(arm),
                new SlideIn(arm),
                new SetJunction(arm, ArmSubsystem.Junction.NONE),
                new MoveToLocation(arm),
                new ReleaseCone(arm)
        );
        /*
        int caseNumber = arm.getCaseNumber();

        addCommands(
                new ReleaseCone(arm),
                new GrabCone(arm)
        );
        Log.d("eencoder", Integer.toString(arm.getLeftEncoderValue()));
        if(caseNumber == 0) {
            Log.d("encoder", Integer.toString(arm.getLeftEncoderValue()));
            addCommands(
                    new SetJunctionPos(arm, -550),
                    new MoveToLocation(arm),
                    new DelayedSlideIn(arm, 1),
                    new SetJunction(arm, ArmSubsystem.Junction.NONE),
                    new MoveToLocation(arm)
            );
            Log.d("condition", "if");
        }
        else if(caseNumber == 1) {
            Log.d("encoder", Integer.toString(arm.getLeftEncoderValue()));;
            addCommands(
                    new DelayedSlideIn(arm, 1),
                    new SetJunction(arm, ArmSubsystem.Junction.NONE),
                    new MoveToLocation(arm)
            );
            Log.d("condition", "else if");
        }
        else {
            Log.d("encoder", Integer.toString(arm.getLeftEncoderValue()));;
            addCommands(
                    new SetJunction(arm, ArmSubsystem.Junction.NONE),
                    new SlideIn(arm)
            );
            Log.d("condition", "else");
        }
        addCommands(
                new ReleaseCone(arm)
        );

        addRequirements(arm);

         */
    }
}
