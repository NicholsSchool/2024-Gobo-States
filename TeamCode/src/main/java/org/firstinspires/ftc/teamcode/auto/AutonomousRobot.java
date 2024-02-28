package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private final ElapsedTime timer;
    private final Drivetrain drivetrain;
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

        //TODO: pot and purple location decide SCORING_Y
        final double SCORING_Y = -36.0;

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        arm = new Arm(hardwareMap, ARM_AUTO_OFFSET, WRIST_AUTO_OFFSET);
        drivetrain = new Drivetrain(hardwareMap, x, y, angle, isBlue);
        hand = new Hand(hardwareMap);
        lights = new Lights(hardwareMap, isBlue);
        lights.setAllianceColor();
        tensorFlowVision = new TensorFlowVision(hardwareMap);

        AnalogInput pot = hardwareMap.get(AnalogInput.class, "pot");
        targetColumn = pot.getVoltage() > pot.getMaxVoltage() * 0.5 ?
                TargetColumn.LEFT : TargetColumn.RIGHT;
        
        this.telemetry = telemetry;

        LerpPath pathOne = isAudience ?
                new LerpPath(new Point(36.0, -36.0), Angles.PI_OVER_TWO) :
                new LerpPath(new Point(-12.0, -36.0), Angles.PI_OVER_TWO);

        LerpPath pathTwo = isAudience ?
                new LerpPath(new Point(48.0, -48.0), 0.0) :
                new LerpPath(new Point(-28.0, -53.0), 0.0);
        LerpPath pathThree = isAudience ?
                new LerpPath(new Point(60.0, -36.0), Angles.PI_OVER_TWO) :
                new LerpPath(new Point(-48.0, SCORING_Y), Angles.PI_OVER_TWO);

        LerpPath pathFour = isAudience ?
                new LerpPath(new Point(46.0, -11), -Math.PI / 4.0 + 0.35) :
                new LerpPath(new Point(-48.0, SCORING_Y), Angles.PI_OVER_TWO);

        LerpPath pathFive = 
                new LerpPath(new Point(-48.0, SCORING_Y), Angles.PI_OVER_TWO);

        if(!isBlue) {
            flipRed(pathOne);
            flipRed(pathTwo);
            flipRed(pathThree);
            flipRed(pathFour);
            flipRed(pathFive);
        }

        lerpPathPlanning = new LerpPathPlanning(drivetrain,
                new LerpPath[]{pathOne, pathTwo, pathThree, pathFour, pathFive});
    }

    private void flipRed(LerpPath path) {
        path.waypoint.y *= -1.0;
        path.angle *= -1.0;

        path.slope = Math.tan(path.angle);

        path.slopePoint = new Point(
                path.waypoint.x + Math.cos(path.angle),
                path.waypoint.y + Math.sin(path.angle));
    }

    /**
     * Updates the Prop Position using the current best recognition.
     * Call within a loop during init()
     */
    public void updatePropLocation() {
        PropLocation currentLocation = tensorFlowVision.getPropLocation();
        if(currentLocation != null)
            propLocation = currentLocation;
        telemetry.addData("Prop Location", propLocation != null ? propLocation : "null");
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
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        timer.reset();
    }

    /**
     * Follows path one while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathOne() {
        arm.armToPosition();
        if(timer.time() > 0.5)
            arm.virtualFourbar();
        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to drop the purple pixel
     */
    public void prepForPurplePixelDrop() {
        arm.setTargetArmPosition(0.0);
        arm.setTargetWristPosition(1400);
        timer.reset();
        //TODO: purple pixel angles and drop
    }

    /**
     * Drops the purple pixel on the correct line
     *
     * @return whether we are finished dropping the pixel
     */
    public boolean dropPurplePixel() {
        arm.armToPosition();
        arm.wristToPosition();
        return timer.time() >= 2.0;
        //TODO: purple pixel angles and drop
    }

    /**
     * Prepares the robot to follow the second auto path
     */
    public void prepForPathTwo() {
        lerpPathPlanning.loadNextPath();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
    }

    /**
     * Follows path two while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathTwo() {
        arm.armToPosition();
        arm.wristToPosition();
        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to follow the third auto path
     */
    public void prepForPathThree() {
        lerpPathPlanning.loadNextPath();
    }

    /**
     * Follows path three while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathThree() {
        arm.armToPosition();
        arm.wristToPosition();
        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to follow the fourth auto path
     */
    public void prepForPathFour() {
        lerpPathPlanning.loadNextPath();
    }

    /**
     * Follows path four while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathFour() {
        arm.armToPosition();
        arm.wristToPosition();
        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to follow the fifth auto path
     */
    public void prepForPathFive() {
        lerpPathPlanning.loadNextPath();
        drivetrain.setTargetHeading(0.0);
    }

    /**
     * Follows path five while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathFive() {
        double robotX = drivetrain.getRobotPose().toPoint().x;
        if(robotX <= -12.0) {
            drivetrain.setTargetHeading(Math.PI);
            arm.setTargetArmPosition(SCORING_POSITION);
            arm.virtualFourbar();
        }
        else {
            arm.wristToPosition();
        }
        arm.armToPosition();
        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to place the yellow pixel
     */
    public void prepForYellowPixelPlace() {
        //TODO: yellow pixel place
    }

    /**
     * Places the Yellow Pixel in the correct column
     *
     * @return whether we are finished placing the pixel
     */
    public boolean placeYellowPixel() {
        return true; //TODO: yellow pixel place
    }

    /**
     * Prepares the robot for setting teleop configuration
     */
    public void prepTeleopConfig() {
        //TODO: yellow pixel place
    }

    /**
     * Sets up the Teleop starting configuration
     *
     * @return whether we are finished setting up teleop
     */
    public boolean configTeleop() {
        return true; //TODO: wrist fully on the ground, arm down, auto align forwards
    }
}