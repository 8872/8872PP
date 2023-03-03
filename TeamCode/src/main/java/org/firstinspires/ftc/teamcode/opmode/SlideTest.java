package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class SlideTest extends BaseOpMode{

    private static final boolean usePhoton = true;
    private static final boolean useBulkread = false;
    protected SlideTest() {
        super(usePhoton, useBulkread);
    }

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
