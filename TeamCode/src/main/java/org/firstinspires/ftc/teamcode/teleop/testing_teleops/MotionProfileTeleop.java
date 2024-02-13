package org.firstinspires.ftc.teamcode.teleop.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;

/**
 * Testing Teleop for Motion Profiles
 */
@Config
@TeleOp(name="Motion Profile Testing", group="Testing")
public class MotionProfileTeleop extends OpMode {
    private ElapsedTime loopTime;
    private Controller controller;
    private MotionProfile motionProfile;
    private VectorMotionProfile vectorMotionProfile;
    private FtcDashboard dashboard;
    public static boolean clip;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        controller = new Controller(gamepad1);
        motionProfile = new MotionProfile(0.5, 0.5);
        vectorMotionProfile = new VectorMotionProfile(0.5);
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

        Vector leftStick = controller.leftStick.toVector();

        if(clip)
            leftStick.clipMagnitude(0.5);
        leftStick = vectorMotionProfile.calculate(leftStick);

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                .setRotation(1.5 * Math.PI)
                .drawGrid(0.0, 0.0, 144.0, 144.0, 21, 21)
                .setFill("red")
                .fillCircle(leftStick.x * 72.0, leftStick.y * 72.0, 5.0);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("left trigger", motionProfile.calculate(controller.leftTrigger.value()));

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