package org.firstinspires.ftc.teamcode.teleop.testing_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math_utils.RobotPose;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;

/**
 * Teleop for Testing April Tag Vision
 */
@Config
@TeleOp(name="April Tag Vision Testing", group="Testing")
public class AprilTagVisionTeleop extends OpMode {
    private ElapsedTime loopTime;
    private FtcDashboard dashboard;
    private AprilTagVision vision;
    private RobotPose robot;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        vision = new AprilTagVision(hardwareMap);
        robot = new RobotPose(0.0, 0.0, 0.0);
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
        RobotPose temp = vision.update();
        if(temp != null)
            robot = temp;

        TelemetryPacket packet = new TelemetryPacket(true);
        packet.fieldOverlay()
                .setFill("green")
                .fillCircle(-robot.x, -robot.y, 8.0)
                .setFill("black")
                .fillCircle(-(robot.x + 8.0 * Math.cos(robot.angle)),
                        -(robot.y + 8.0 * Math.sin(robot.angle)), 2.0);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("pose", robot);
        telemetry.addData("number of detections", vision.getNumDetections());
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