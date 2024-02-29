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
import org.firstinspires.ftc.teamcode.math_utils.Vector;
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
    private final boolean isBlueAlliance;
    private double dropAngle;
    private double placeAngle;


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

        this.isBlueAlliance = isBlue;

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

        //TODO: pot and purple location decide SCORING_Y
        final double SCORING_X = -50.0;
        final double SCORING_Y = -36.0;

        LerpPath pathOne = isAudience ?
                new LerpPath(new Point(36.0, -36.0), Angles.PI_OVER_TWO) :
                new LerpPath(new Point(-12.0, -36.0), Angles.PI_OVER_TWO);

        LerpPath pathTwo = isAudience ?
                new LerpPath(new Point(48.0, -48.0), 0.0) :
                new LerpPath(new Point(-28.0, -53.0), 0.0);
        LerpPath pathThree = isAudience ?
                new LerpPath(new Point(60.0, -36.0), Angles.PI_OVER_TWO) :
                new LerpPath(new Point(SCORING_X, SCORING_Y), Angles.PI_OVER_TWO);

        LerpPath pathFour = new LerpPath(new Point(46.0, -11.0), -Math.PI / 4.0 + 0.35);

        LerpPath pathFive = new LerpPath(new Point(-12.0, -11.0), 0.0);

        LerpPath pathSix = new LerpPath(new Point(SCORING_X, SCORING_Y), Angles.PI_OVER_TWO);

        if(!isBlue) {
            flipRed(pathOne);
            flipRed(pathTwo);
            flipRed(pathThree);
            flipRed(pathFour);
            flipRed(pathFive);
            flipRed(pathSix);
        }

        lerpPathPlanning = new LerpPathPlanning(drivetrain,
                new LerpPath[]{pathOne, pathTwo, pathThree, pathFour, pathFive, pathSix});
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
     * Closes the Vision Portal and finalizes prop related data
     */
    public void closeVision() {
        tensorFlowVision.close();

        if(propLocation == null)
            propLocation = PropLocation.CENTER;

        if(propLocation == PropLocation.LEFT) {
            dropAngle = Angles.PI_OVER_TWO;
            placeAngle = Math.PI;
        }
        else if(propLocation == PropLocation.CENTER) {
            dropAngle = Math.PI / 6.0;
            placeAngle = Angles.PI_OVER_TWO;
        }
        else {
            dropAngle = Angles.PI_OVER_TWO;
            placeAngle = Math.PI / 6.0;
        }

        if(!isBlueAlliance) {
            dropAngle *= -1.0;
            placeAngle += Math.PI;
        }

    }

    /**
     * Prepares the robot to follow the first auto path
     */
    public void prepForPathOne() {
        drivetrain.setTargetHeading(dropAngle);
        arm.setTargetArmPosition(1500.0);
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
        if(timer.time() > 0.75)
            arm.virtualFourbar();
        else
            arm.wristManual(0.0);

        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to drop the purple pixel
     */
    public void prepForPurplePixelDrop() {
        drivetrain.setTargetHeading(placeAngle);
        arm.setTargetArmPosition(0.0);
        timer.reset();
    }

    /**
     * Drops the purple pixel on the correct line
     *
     * @return whether we are finished dropping the pixel
     */
    public boolean dropPurplePixel() {
        drivetrain.update();
        arm.armToPosition();
        arm.virtualFourbar();

        boolean autoAlign = timer.time() >= 1.0;

        drivetrain.drive(new Vector(0.0, 0.0), 0.0, autoAlign, true);
        if(Math.abs(drivetrain.getRobotPose().angle - placeAngle) <= 0.0174532) {
            hand.toggleRight();
            arm.setTargetWristPosition(1400.0);
            timer.reset();
            return true;
        }
        return false;
    }

    /**
     * Pauses the course of the auto after placing the purple pixel
     *
     * @return if the duration is over
     */
    public boolean waitAfterPurple() {
        if(timer.time() <= 0.25)
            arm.virtualFourbar();
        else
            arm.wristToPosition();

        drivetrain.update();
        return timer.time() >= 1.0;
    }

    /**
     * Prepares the robot to follow the second auto path
     */
    public void prepForPathTwo() {
        drivetrain.setTargetHeading(isBlueAlliance ? Angles.NEGATIVE_PI_OVER_TWO : Angles.PI_OVER_TWO);
        hand.toggleRight();
        lerpPathPlanning.loadNextPath();
        arm.setTargetWristPosition(1400);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
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
        arm.wristToPosition();
        arm.armToPosition();
        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to follow the fifth auto path
     */
    public void prepForPathSix() {
        lerpPathPlanning.loadNextPath();
        drivetrain.setTargetHeading(Math.PI);
        arm.setTargetArmPosition(SCORING_POSITION - 200.0);
    }

    /**
     * Follows path five while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathSix() {
        arm.virtualFourbar();
        arm.armToPosition();
        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to place the yellow pixel
     */
    public void prepForYellowPixelPlace() {
        hand.toggleLeft();
        timer.reset();
    }

    /**
     * Places the Yellow Pixel in the correct column
     *
     * @return whether we are finished placing the pixel
     */
    public boolean placeYellowPixel() {
        drivetrain.update();
        double targetAngle = isBlueAlliance ? Angles.PI_OVER_TWO : Angles.NEGATIVE_PI_OVER_TWO;

        double time = timer.time();
        if(time >= 0.5) {
            drivetrain.setTargetHeading(targetAngle);
        }
        if(time >= 1.5)
            arm.setTargetArmPosition(0.0);

        if(time >= 2.0) {
            arm.setTargetWristPosition(-50.0);
            arm.wristToPosition();
        }
        else
            arm.wristManual(0.0);

        arm.armToPosition();
        drivetrain.drive(new Vector(0.0, 0.0), 0.0, true, true);

        return Math.abs(drivetrain.getRobotPose().angle - targetAngle) <= 0.0174532;
    }
}