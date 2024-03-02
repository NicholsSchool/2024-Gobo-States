package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
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
public class AutonomousRobot implements ArmConstants, SplineConstants, DrivetrainConstants {
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
    private LerpPathPlanning lerpPathPlanning;
    private final Telemetry telemetry;
    private final TargetColumn targetColumn;
    private PropLocation propLocation;
    private final boolean isBlueAlliance;
    private final boolean isAudience;
    private Point scoringPoint;
    private double dropAngle;
    private double placeAngle;
    private boolean alreadyClosed;


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
        this.isAudience = isAudience;

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
            placeAngle = Math.PI * 8.0 / 9.0;
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

        double SCORING_Y = -35.5;
        if(propLocation == PropLocation.LEFT)
            SCORING_Y += isBlueAlliance ? -6.0 : 6.0;
        else if(propLocation == PropLocation.RIGHT)
            SCORING_Y += isBlueAlliance ? 6.0 : -6.0;

        if(targetColumn == TargetColumn.LEFT)
            SCORING_Y += isBlueAlliance ? -1.75 : 1.75;
        else if(targetColumn == TargetColumn.RIGHT)
            SCORING_Y += isBlueAlliance ? 1.75 : -1.75;

        final double SCORING_X = -48.0;

        LerpPath pathOne = isAudience ?
                new LerpPath(new Point(36.0, -36.0), Angles.PI_OVER_TWO) :
                new LerpPath(new Point(-12.0, -36.0), Angles.PI_OVER_TWO);

        LerpPath pathTwo = isAudience ?
                new LerpPath(new Point(48.0, -48.0), 0.0) :
                new LerpPath(new Point(-28.0, -53.0), 0.0);
        LerpPath pathThree = isAudience ?
                new LerpPath(new Point(58.0, -36.0), Angles.PI_OVER_TWO) :
                new LerpPath(new Point(SCORING_X, SCORING_Y), Angles.PI_OVER_TWO);

        LerpPath pathFour = isAudience ?
                new LerpPath(new Point(46.0, -11.0), -Math.PI / 4.0 + 0.35) :
                new LerpPath(new Point(SCORING_X, SCORING_Y), Angles.PI_OVER_TWO);

        LerpPath pathFive = isAudience ?
                new LerpPath(new Point(-12.0, -11.0), 0.0) :
                new LerpPath(new Point(SCORING_X, SCORING_Y), Angles.PI_OVER_TWO);

        LerpPath pathSix = new LerpPath(new Point(SCORING_X, SCORING_Y), Angles.PI_OVER_TWO);

        if(!isBlueAlliance) {
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
        arm.setTargetWristPosition(-50.0);
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
        arm.wristToPosition();

        boolean autoAlign = timer.time() >= 1.0;

        drivetrain.drive(new Vector(0.0, 0.0), 0.0, autoAlign, true);

        if(Math.abs(Angles.clipRadians(drivetrain.getRobotPose().angle - placeAngle))
                <= AUTO_ALIGN_ERROR) {
            hand.toggleRight();
            hand.toggleLeft();
            arm.setTargetWristPosition(-50.0);
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
        if(timer.time() >= 0.25 && !alreadyClosed) {
            alreadyClosed = true;
            hand.toggleLeft();
        }

        if(timer.time() >= 0.6)
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
        drivetrain.setTargetHeading(isBlueAlliance ?
            -Math.PI / 4.0 : Math.PI / 4.0);
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
        if(!isAudience) {
            drivetrain.setTargetHeading(Math.PI);
            arm.setTargetArmPosition(SCORING_POSITION - 550.0);
        }
        lerpPathPlanning.loadNextPath();
    }

    /**
     * Follows path three while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathThree() {
        arm.armToPosition();

        if(isAudience)
            arm.wristToPosition();
        else
            arm.virtualFourbar();

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

        if(isAudience)
            arm.wristToPosition();
        else
            arm.virtualFourbar();

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

        if(isAudience)
            arm.wristToPosition();
        else
            arm.virtualFourbar();

        return lerpPathPlanning.spline(0.0, true, true);
    }

    /**
     * Prepares the robot to follow the fifth auto path
     */
    public void prepForPathSix() {
        lerpPathPlanning.loadNextPath();
        drivetrain.setTargetHeading(Math.PI);
        arm.setTargetArmPosition(SCORING_POSITION - 550.0);
        timer.reset();
    }

    /**
     * Follows path five while moving the arm and wrist appropriately
     *
     * @return whether we have reached the path's destination
     */
    public boolean followPathSix() {
        arm.virtualFourbar();
        arm.armToPosition();
        boolean autoAlign = timer.time() >= 0.25;
        return lerpPathPlanning.spline(isBlueAlliance ? 1.0 : -1.0, autoAlign, true);
    }

    public void prepForNudge() {
        scoringPoint = new Point(-100.0, drivetrain.getRobotPose().y);
        timer.reset();
    }

    public boolean nudgeForward() {
        Vector driveVector = drivetrain.getRobotPose().toPoint().slope(scoringPoint);

        driveVector.scaleMagnitude(1.0);

        drivetrain.drive(driveVector, 0.0, true, true);
        return timer.time() >= 0.25;
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
        if(time <= 0.4) {
            drivetrain.drive(new Vector(1.0, 0.0), 0.0, true, false);
        }
        else
            drivetrain.drive(new Vector(0.0, 0.0), 0.0, true, true);

        if(time >= 1.0) {
            arm.setTargetArmPosition(0.0);
            drivetrain.setTargetHeading(targetAngle);
        }

        if(time >= 1.5) {
            arm.setTargetWristPosition(-50.0);
            arm.wristToPosition();
        }
        else
            arm.wristManual(0.0);

        arm.armToPosition();

        return Math.abs(Angles.clipRadians(drivetrain.getRobotPose().angle - targetAngle))
                <= AUTO_ALIGN_ERROR;
    }
}