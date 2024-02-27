package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.LerpPath;
import org.firstinspires.ftc.teamcode.math_utils.LerpPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.TensorFlowVision;
import org.firstinspires.ftc.teamcode.subsystems.TensorFlowVision.PropLocation;

/**
 * Robot Container for the Autonomous Period
 */
public class AutonomousRobot implements ArmConstants, SplineConstants {
    /**
     * The two target columns for autonomous
     */
    public enum TargetColumn {
        LEFT, RIGHT
    }

    private final Arm arm;
    private final Hand hand;
    private final Lights lights;
    private final TensorFlowVision tensorFlowVision;
    private final LerpPathPlanning lerpPathPlanning;
    private final Telemetry telemetry;
    private final TargetColumn targetColumn;
    private PropLocation propLocation;

    /**
     * @param hardwareMap the hardware map
     * @param x the initial x
     * @param y the initial y
     * @param angle the initial angle
     * @param isBlue whether we are blue alliance
     * @param isAudience whether we are on the audience side
     * @param telemetry the telemetry
     */
    public AutonomousRobot(HardwareMap hardwareMap, double x, double y, double angle,
                           boolean isBlue, boolean isAudience, Telemetry telemetry) {

        arm = new Arm(hardwareMap, ARM_AUTO_OFFSET, WRIST_AUTO_OFFSET);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, x, y, angle, isBlue);
        hand = new Hand(hardwareMap);
        lights = new Lights(hardwareMap, isBlue);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        tensorFlowVision = new TensorFlowVision(hardwareMap);

        AnalogInput pot = hardwareMap.get(AnalogInput.class, "pot");
        targetColumn = pot.getVoltage() > pot.getMaxVoltage() * 0.5 ?
                TargetColumn.LEFT : TargetColumn.RIGHT;
        
        this.telemetry = telemetry;

        LerpPath pathOne = isAudience ?
                new LerpPath(new Point(36.0, -36.0), Angles.PI_OVER_TWO) :
                new LerpPath(new Point(-12.0, -36.0), Angles.PI_OVER_TWO);

        if(!isBlue) {
            pathOne.waypoint.y *= -1.0;
            pathOne.angle *= -1.0;

            pathOne.slope = Math.tan(angle);

            pathOne.slopePoint = new Point(
                    pathOne.waypoint.x + Math.cos(angle),
                    pathOne.waypoint.y + Math.sin(angle));
        }

        lerpPathPlanning = new LerpPathPlanning(drivetrain, new LerpPath[]{pathOne});
    }

    /**
     * Updates the Prop Position using the current best recognition.
     * Call within a loop during init()
     */
    public void updatePropLocation() {
        PropLocation currentLocation = tensorFlowVision.getPropLocation();
        if(currentLocation != null)
            propLocation = currentLocation;
        telemetry.addData("Prop Location", propLocation);
        telemetry.update();
    }

    /**
     * Closes the Vision Portal
     */
    public void closeVision() {
        tensorFlowVision.close();

        if(propLocation == null)
            propLocation = PropLocation.CENTER;
    }

    /**
     * Prepares the robot to follow the first auto path
     */
    public void prepForPathOne() {
        arm.setTargetArmPosition(1400.0);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
    }

    /**
     * Follows path one while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathOne() {
        arm.armToPosition();
        arm.virtualFourbar();
        return lerpPathPlanning.spline(0.0, true, true);
    }
}