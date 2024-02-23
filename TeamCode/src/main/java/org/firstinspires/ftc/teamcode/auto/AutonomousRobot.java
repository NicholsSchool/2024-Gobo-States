package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.math_utils.BezierPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.CubicBezierPath;
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
    private final BezierPathPlanning bezierPathPlanning;
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

        AnalogInput pot = hardwareMap.get(AnalogInput.class, "pot");
        targetColumn = pot.getVoltage() > pot.getMaxVoltage() * 0.5 ? TargetColumn.LEFT : TargetColumn.RIGHT;
        
        this.telemetry = telemetry;

        
        
        //TODO: test that the new Bezier Spline class works in a new testing auto
        
        //Here are two example paths that are determined by our starting side in auto and our alliance color
        //TODO: make all the real paths for both audience and backstage side
        CubicBezierPath exampleOne = isAudience ?
                new CubicBezierPath(new Point(0, 0), new Point(1, 1),
                                    new Point(2, 2), new Point(3, 3)) :
                new CubicBezierPath(new Point(69, 69), new Point(70, 70),
                                    new Point(71, 71), new Point(72, 72));

        CubicBezierPath exampleTwo = isAudience ?
                new CubicBezierPath(new Point(-10, -10), new Point(-11, -11),
                        new Point(-12, -12), new Point(-13, -13)) :
                new CubicBezierPath(new Point(0, 1), new Point(2, 3),
                        new Point(4, 5), new Point(6, 7));

        if(!isBlue) {
            //flips the y values if we are red
            exampleOne.one.y *= -1;
            exampleOne.two.y *= -1;
            exampleOne.three.y *= -1;
            exampleOne.four.y *= -1;

            exampleTwo.one.y *= -1;
            exampleTwo.two.y *= -1;
            exampleTwo.three.y *= -1;
            exampleTwo.four.y *= -1;
        }

        //TODO: tune correction distance and number of steps in SplineConstants
        bezierPathPlanning = new BezierPathPlanning(drivetrain,
                new CubicBezierPath[]{exampleOne, exampleTwo}, //Puts all the paths into BezierPathPlanning
                CORRECTION_DISTANCE, STEPS);
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
    public void closeVision() {
        tensorFlowVision.close();

        if(propLocation == null)
            propLocation = PropLocation.CENTER;
    }



    //Here are demo methods for preparing and executing each step of the auto
    //See how they are used in the actual Auto classes
    //TODO: make methods for all parts of the auto using this structure

    /**
     * Prepares the robot to follow path one
     */
    public void prepForPathOne() {
        arm.setTargetArmPosition(1400.0);
        lights.setAllianceColor();
    }

    /**
     * Follows path one while moving the arm and wrist
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathOne() {
        arm.armToPosition();
        arm.virtualFourbar();
        return bezierPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to follow path two
     */
    public void prepForPathTwo() {
        arm.setTargetArmPosition(0.0);
        lights.setAllianceColor();
        bezierPathPlanning.loadNextPath();
    }

    /**
     * Follows path two while moving the arm and wrist
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathTwo() {
        arm.armToPosition();
        arm.virtualFourbar();
        return bezierPathPlanning.spline(0.0, true, true);
    }
}