package org.firstinspires.ftc.teamcode.teleop.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

/**
 * Testing for Lights
 */
@Config
@TeleOp(name="Lights Testing", group="Testing")
public class LightsTeleop extends OpMode implements RobotConstants {
    private Lights lights;
    private ElapsedTime loopTime;
    private FtcDashboard dashboard;
    public static boolean turnGold;
    public static boolean turnWhite;
    public static boolean turnYellow;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        lights = new Lights(hardwareMap, IS_RED_ALLIANCE);
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
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

    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(turnGold)
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        else if(turnWhite)
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        else if(turnYellow)
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        else
            lights.setAllianceColor();

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