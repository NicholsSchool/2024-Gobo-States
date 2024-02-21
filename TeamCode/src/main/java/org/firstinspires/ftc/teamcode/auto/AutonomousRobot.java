package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.math_utils.BezierPathPlanning;
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
    private final Drivetrain drivetrain;
    private final Hand hand;
    private final Lights lights;
    private final TensorFlowVision tensorFlowVision;
    private final AnalogInput pot;
    private final HardwareMap hardwareMap;
    private final BezierPathPlanning splineOne;
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
        drivetrain = new Drivetrain(hardwareMap, x, y, angle, isBlue);
        hand = new Hand(hardwareMap);
        lights = new Lights(hardwareMap, isBlue);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        tensorFlowVision = new TensorFlowVision(hardwareMap);

        pot = hardwareMap.get(AnalogInput.class, "pot");
        targetColumn = pot.getVoltage() > pot.getMaxVoltage() * 0.5 ? TargetColumn.LEFT : TargetColumn.RIGHT;

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;



        //TODO: real point arrays for real paths, and several for the whole auto
        //example point arrays for bezier based off isAudience, just switch with the real points
        Point[] splineOnePoints = isAudience ?
                new Point[]{new Point(0, 0), new Point(1, 1)} :
                new Point[]{new Point(10, 10), new Point(11, 11)};

        //TODO: tune correction distance and steps in SplineConstants
        //TODO: test that the new Bezier Spline class works in a testing auto
        splineOne = new BezierPathPlanning(drivetrain, splineOnePoints, CORRECTION_DISTANCE, STEPS);

        if(!isBlue) {
            for(Point point : splineOnePoints) {
                point.y *= -1; //mirrors y value for red alliance
            }
        }
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
    }

    /**
     * Closes the Vision Portal
     */
    public void startAutoRoutine() {
        tensorFlowVision.close();

        if(propLocation == null)
            propLocation = PropLocation.CENTER;

        arm.setTargetArmPosition(1400.0);
        lights.setAllianceColor();
    }

    //DEMO method to base real ones off of
    //TODO: make real paths and methods using this structure
    public boolean followPathOne() {
        arm.armToPosition();
        arm.virtualFourbar();
        return splineOne.spline(0.0, true, true);
    }
}