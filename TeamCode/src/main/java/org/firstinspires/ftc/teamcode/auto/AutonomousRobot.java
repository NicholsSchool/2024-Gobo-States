package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.TensorFlowVision;

/**
 * Robot Container for the Autonomous Period
 */
public class AutonomousRobot {
    private final ElapsedTime timer;
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Hand hand;
    private final Lights lights;
    private final TensorFlowVision tensorFlowVision;
    private AprilTagVision aprilTagVision;

    /**
     * @param hwMap the hardware map
     * @param x the initial x
     * @param y the initial y
     * @param angle the initial angle
     * @param isBlue whether we are blue alliance
     */
    public AutonomousRobot(HardwareMap hwMap, double x, double y, double angle, boolean isBlue) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        arm = new Arm(hwMap, 0, 0);
        drivetrain = new Drivetrain(hwMap, x, y, angle);
        hand = new Hand(hwMap);
        lights = new Lights(hwMap, isBlue);
        tensorFlowVision = new TensorFlowVision(hwMap);
    }
}