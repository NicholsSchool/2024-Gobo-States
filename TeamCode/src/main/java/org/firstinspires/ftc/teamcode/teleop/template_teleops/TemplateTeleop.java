package org.firstinspires.ftc.teamcode.teleop.template_teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Sample Teleop
 */
@TeleOp(name="Template", group="Template")
public class TemplateTeleop extends OpMode {
    private ElapsedTime loopTime;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        loopTime.reset();
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("loop time", loopTime.time());
        loopTime.reset();
        telemetry.update();
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}