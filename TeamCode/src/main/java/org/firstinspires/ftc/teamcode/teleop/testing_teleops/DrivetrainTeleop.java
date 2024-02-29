package org.firstinspires.ftc.teamcode.teleop.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Teleop for the Drivetrain
 */
@Config
@TeleOp(name="Drivetrain Testing", group="Testing")
public class DrivetrainTeleop extends OpMode implements RobotConstants {
    private Drivetrain drivetrain;
    private Controller controller;
    private ElapsedTime loopTime;
    private FtcDashboard dashboard;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        controller = new Controller(gamepad1);
        drivetrain = new Drivetrain(hardwareMap, 0.0, 0.0, Angles.PI_OVER_TWO, IS_BLUE_ALLIANCE);
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetryNavx();
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
        controller.update();
        drivetrain.update();

        if(controller.triangle.wasJustPressed())
            drivetrain.setTargetHeading(Angles.PI_OVER_TWO);
        else if(controller.x.wasJustPressed())
            drivetrain.setTargetHeading(Angles.NEGATIVE_PI_OVER_TWO);
        else if(controller.circle.wasJustPressed())
            drivetrain.setTargetHeading(0.0);
        else if(controller.square.wasJustPressed())
            drivetrain.setTargetHeading(Math.PI);

        drivetrain.drive(
                controller.leftStick.toVector(),
                controller.rightStick.x.value(),
                controller.rightStick.x.hasBeenZero(),
                controller.leftTrigger.value() <= 0.0);

        telemetryNavx();

        telemetry.addData("pose", drivetrain.getRobotPose());

        telemetry.addData("loop time", loopTime.time());
        loopTime.reset();
        telemetry.update();
    }

    private void telemetryNavx() {
        boolean[] navxInfo = drivetrain.getNavxInfo();
        double[] navxAves = drivetrain.getNavxAxes();
        telemetry.addData("is connected", navxInfo[0]);
        telemetry.addData("is calibrating", navxInfo[1]);
        telemetry.addData("pitch", navxAves[0]);
        telemetry.addData("roll", navxAves[1]);
        telemetry.addData("yaw", navxAves[2]);
        telemetry.update();
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}