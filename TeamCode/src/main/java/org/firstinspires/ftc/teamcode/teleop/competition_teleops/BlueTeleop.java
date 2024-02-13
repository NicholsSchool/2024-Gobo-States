package org.firstinspires.ftc.teamcode.teleop.competition_teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.teleop.TeleopRobot;

/**
 * Blue Competition Teleop
 */
@TeleOp(name="Blue", group="Competition")
public class BlueTeleop extends OpMode implements RobotConstants {
    private ElapsedTime loopTime;
    private TeleopRobot robot;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot = new TeleopRobot(hardwareMap,
                0.0, 0.0, Angles.PI_OVER_TWO, IS_BLUE_ALLIANCE, gamepad1, gamepad2);
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
        robot.update();
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