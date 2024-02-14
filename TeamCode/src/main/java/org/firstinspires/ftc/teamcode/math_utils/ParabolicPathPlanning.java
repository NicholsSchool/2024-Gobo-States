package org.firstinspires.ftc.teamcode.math_utils;

import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Math for Parabolic Path Planning
 */
public class ParabolicPathPlanning implements SplineConstants {
    private final Drivetrain drivetrain;
    private final double WAYPOINT_Y;
    private final Point INTAKE;
    private final Point PURPLE_DROP;

    /**
     * Instantiates the Parabolic Spline
     *
     * @param drivetrain the drivetrain
     * @param isBlueAlliance whether we are blue alliance
     */
    public ParabolicPathPlanning(Drivetrain drivetrain, boolean isBlueAlliance, boolean isAudience) {
        this.drivetrain = drivetrain;
        WAYPOINT_Y = isBlueAlliance ? BLUE_WAYPOINT_Y : RED_WAYPOINT_Y;
        INTAKE = new Point(INTAKE_X, isBlueAlliance ? BLUE_INTAKE_Y : RED_INTAKE_Y);
        PURPLE_DROP = new Point(isAudience ? 36.0 : -36.0, isBlueAlliance ? -36.0 : 36.0);

    }

    /**
     * With the robot at (rx, ry), calculates the drive vector of the robot
     * in order to follow a parabola and arrive at the waypoint (wx, wy)
     * that is the parabola's vertex.
     * The parabola is defined to contain the robot
     *
     * @param robot the robot (rx, ry)
     * @param waypoint the waypoint (wx, wy)
     * @param toIntake whether the robot is going to the intake
     *
     * @return the unscaled drive vector in [x, y] notation
     */
    public Vector vectorToVertex(Point robot, Point waypoint, boolean toIntake) {
        if(robot.x == waypoint.x)
            return toIntake ? new Vector(1.0, 0.0) : new Vector(-1.0, 0.0);

        return new Vector(waypoint.x - robot.x, (waypoint.y - robot.y) * 2.0);
    }

    /**
     * With the robot at (x, y), calculates the drive vector of the robot
     * in order to follow a parabola and arrive at the waypoint (wx, wy).
     * The parabola is defined with its vertex constrained to the x-value
     * of h (the previous waypoint), and the curve consists of both the
     * waypoint and robot coordinates.
     *
     * @param robot the robot (rx, ry)
     * @param waypoint the waypoint (wx, wy)
     * @param h  the x value of the previous waypoint
     * @param toIntake whether the robot is going to the intake
     *
     * @return the unscaled drive vector in [x, y] notation
     */
    public Vector vectorFromVertex(Point robot, Point waypoint, double h, boolean toIntake) {
        if(robot.x == waypoint.x)
            return toIntake ? new Vector(1.0, 0.0) : new Vector(-1.0, 0.0);

        double robotDistSquared = Math.pow(robot.x - h, 2);
        double waypointDistSquared = Math.pow(waypoint.x - h, 2);

        if(robotDistSquared == waypointDistSquared)
            return robot.y < waypoint.y ? new Vector(0.0, 1.0) : new Vector(0.0, -1.0);

        double k = (waypoint.y * robotDistSquared - robot.y * waypointDistSquared)
                / (robotDistSquared - waypointDistSquared);

        return (robot.x > waypoint.x) == toIntake ?
                new Vector(h - robot.x, (k - robot.y) * 2.0) :
                new Vector(robot.x - h, (robot.y - k) * 2.0);
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct Intake
     * area using parabolas in piecewise.
     *
     * @param turn the turn speed proportion
     * @param autoAlign whether to autoAlign
     * @param lowGear whether to drive in low gear
     *
     * @return if we are close enough to the destination area
     */
    public boolean splineToIntake(double turn, boolean autoAlign, boolean lowGear) {
        Point robot = drivetrain.getRobotPose().toPoint();

        Vector drive;
        if(robot.x < LEFT_WAYPOINT_X)
            drive = vectorToVertex(robot, new Point(LEFT_WAYPOINT_X, WAYPOINT_Y), true);
        else if(robot.x < RIGHT_WAYPOINT_X)
            drive = vectorToVertex(robot, new Point(RIGHT_WAYPOINT_X, WAYPOINT_Y), true);
        else
            drive = vectorFromVertex(robot, INTAKE, RIGHT_WAYPOINT_X, true);

        double distance = robot.distance(INTAKE);

        boolean isFinished;
        if(distance >= SPLINE_ERROR) {
            drive.scaleMagnitude(SPLINE_P * distance);
            isFinished = false;
        }
        else {
            drive.zero();
            isFinished = true;
        }

        drivetrain.drive(drive, turn, autoAlign, lowGear);
        return isFinished;
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct Backstage
     * area using parabolas in piecewise.
     *
     * @param turn the turn speed proportion
     * @param autoAlign whether to autoAlign
     * @param scoringY the Y value to end at
     * @param lowGear whether to drive in low gear
     *
     * @return if we are close enough to the destination area
     */
    public boolean splineToScoring(double turn, boolean autoAlign, double scoringY, boolean lowGear) {
        Point robot = drivetrain.getRobotPose().toPoint();
        Point scoring = new Point(SCORING_X, scoringY);

        Vector drive;
        if(robot.x > RIGHT_WAYPOINT_X)
            drive = vectorToVertex(robot, new Point(RIGHT_WAYPOINT_X, CENTER_WAYPOINT_Y), false);
        else if(robot.x > LEFT_WAYPOINT_X)
            drive = vectorToVertex(robot, new Point(LEFT_WAYPOINT_X, CENTER_WAYPOINT_Y), false);
        else
            drive = vectorFromVertex(robot, scoring, LEFT_WAYPOINT_X, false);

        double distance = robot.distance(scoring);

        boolean isFinished;
        if(distance >= SPLINE_ERROR) {
            drive.scaleMagnitude(SPLINE_P * distance);
            isFinished = false;
        }
        else {
            drive.zero();
            isFinished = true;
        }

        drivetrain.drive(drive, turn, autoAlign, lowGear);
        return isFinished;
    }

    /**
     * Automatically directs the robot to the Coordinates of the Correct
     * purple pixel drop location using a single parabola
     *
     * @param turn the turn speed proportion
     * @param autoAlign whether to autoAlign
     * @param lowGear whether to drive in low gear
     *
     * @return if we are close enough to the destination area
     */
    public boolean splineToPurple(double turn, boolean autoAlign, boolean lowGear) {
        Point robot = drivetrain.getRobotPose().toPoint();

        Vector drive = vectorToVertex(robot, PURPLE_DROP, true);

        double distance = robot.distance(PURPLE_DROP);

        boolean isFinished;
        if(distance >= SPLINE_ERROR) {
            drive.scaleMagnitude(SPLINE_P * distance);
            isFinished = false;
        }
        else {
            drive.zero();
            isFinished = true;
        }

        drivetrain.drive(drive, turn, autoAlign, lowGear);
        return isFinished;
    }
}