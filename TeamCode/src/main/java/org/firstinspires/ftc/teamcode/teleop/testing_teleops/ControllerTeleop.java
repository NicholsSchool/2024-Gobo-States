package org.firstinspires.ftc.teamcode.teleop.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.Vector;

/**
 * Testing Teleop for Controller
 */
@Config
@TeleOp(name="Controller Testing", group="Testing")
public class ControllerTeleop extends OpMode {
    private ElapsedTime loopTime;
    private Controller controller;
    private FtcDashboard dashboard;
    private boolean squareToggle;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        controller = new Controller(gamepad1);
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
        controller.update();

        squareToggle = controller.square.wasJustPressed() != squareToggle;

        telemetry.addData("BUTTONS", "");
        telemetry.addData("x", controller.x);
        telemetry.addData("circle", controller.circle);
        telemetry.addData("square TOGGLE", squareToggle);
        telemetry.addData("triangle", controller.triangle);
        telemetry.addData("options", controller.options);
        telemetry.addData("share", controller.share);
        telemetry.addData("dpad up", controller.dpadUp);
        telemetry.addData("dpad down", controller.dpadDown);
        telemetry.addData("dpad left", controller.dpadLeft);
        telemetry.addData("dpad right", controller.dpadRight);
        telemetry.addData("left stick button", controller.leftStickButton);
        telemetry.addData("right stick button", controller.rightStickButton);

        telemetry.addData("AXES", "");
        telemetry.addData("left trigger", controller.leftTrigger.value());
        telemetry.addData("right trigger zeroed", controller.rightTrigger.hasBeenZero());

        Vector leftStick = controller.leftStick.toVector();
        Vector rightStick = controller.rightStick.toVector();

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                .setRotation(1.5 * Math.PI)
                .drawGrid(0.0, 0.0, 144.0, 144.0, 21, 21)
                .setFill("red")
                .fillCircle(leftStick.x * 72.0, leftStick.y * 72.0, 5.0)
                .setFill("green")
                .fillCircle(rightStick.x * 72.0, rightStick.y * 72.0, 5.0);
        dashboard.sendTelemetryPacket(packet);

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