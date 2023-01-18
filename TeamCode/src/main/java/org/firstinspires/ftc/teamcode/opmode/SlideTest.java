package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class SlideTest extends BaseOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        initialize();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) slide.out();
            if (gamepad1.b) slide.in();
        }
    }
}
