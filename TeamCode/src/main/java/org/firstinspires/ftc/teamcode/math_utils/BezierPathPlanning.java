package org.firstinspires.ftc.teamcode.math_utils;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.stream.DoubleStream;

/**
 * Math for Parabolic Path Planning
 */
public class BezierPathPlanning implements SplineConstants {
    private final Drivetrain drivetrain;
    private final CubicBezierPath[] bezierPaths;
    private CubicBezierPath currentPath;
    private int pathIndex;
    private final double correctionDistance;
    private final int steps;
    private Point robotPosition;

    /**
     * Instantiates the BezierSpline
     *
     * @param drivetrain the drivetrain
     * @param paths the array of bezier paths to be used
     * @param correctionDistance the distance threshold for correction
     * @param steps the number of distance samples to take
     */
    public BezierPathPlanning(
            Drivetrain drivetrain, CubicBezierPath[] paths, double correctionDistance, int steps) {
        this.drivetrain = drivetrain;
        bezierPaths = paths;
        currentPath = bezierPaths[pathIndex];
        this.correctionDistance = correctionDistance;
        this.steps = steps;
        robotPosition = drivetrain.getRobotPose().toPoint();
    }

    /**
     * Switches the current path to the next one
     * if there is another
     */
    public void loadNextPath() {
        pathIndex++;
        if(pathIndex < bezierPaths.length)
            currentPath = bezierPaths[pathIndex];
    }

    private double bezierX(double t) {
        return Math.pow(1 - t, 3) * currentPath.one.x + 3 * t * Math.pow(1 - t, 2) * currentPath.two.x +
                3 * Math.pow(t, 2) * (1 - t) * currentPath.three.x + Math.pow(t, 3) * currentPath.four.x;
    }

    private double bezierY(double t) {
        return Math.pow(1 - t, 3) * currentPath.one.y + 3 * t * Math.pow(1 - t, 2) * currentPath.two.y +
                3 * Math.pow(t, 2) * (1 - t) * currentPath.three.y + Math.pow(t, 3) * currentPath.four.y;
    }

    private Vector tangentVector(double t) {
        double firstCoefficient = Math.pow(1 - t, 2);
        double secondCoefficient = 2 * t * (1 - t);
        double thirdCoefficient = Math.pow(t, 2);

        Vector tangentVector = new Vector(
                firstCoefficient * (currentPath.two.x - currentPath.one.x) +
                        secondCoefficient * (currentPath.three.x - currentPath.two.x) +
                        thirdCoefficient * currentPath.four.x - currentPath.three.x,
                firstCoefficient * (currentPath.two.y - currentPath.one.y) +
                        secondCoefficient * (currentPath.three.y - currentPath.two.y) +
                        thirdCoefficient * currentPath.four.y - currentPath.three.y);

        tangentVector.scaleMagnitude(1.0);
        return tangentVector;
    }

    private double distance(double t) {
        return Math.hypot(bezierX(t) - robotPosition.x, bezierY(t) - robotPosition.y);
    }

    private double desiredT() {
        double height = distance(1.0);

        double desiredT = 0.0;
        double increment = 1.0 / steps;

        double[] intersections = DoubleStream.iterate(0.0, n -> n + increment).limit(steps).toArray();

        for(double intersection : intersections) {
            double thisHeight = distance(intersection);
            if (thisHeight < height) {
                height = thisHeight;
                desiredT = intersection;
            }
        }
        return desiredT;
    }

    /**
     * With the robot at (x, y), calculates the drive vector of the robot
     * in order to follow a set Bezier curve
     *
     * @param turn the turn speed proportion
     * @param autoAlign whether to autoAlign
     * @param lowGear whether to drive in low gear
     *
     * @return if we are close enough to the destination area
     */
    public boolean spline(double turn, boolean autoAlign, boolean lowGear) {
        drivetrain.update();
        robotPosition = drivetrain.getRobotPose().toPoint();

        double error = robotPosition.distance(currentPath.four);

        Vector drive;
        if(error <= correctionDistance) {
            drive = robotPosition.slope(currentPath.four);
        }
        else {
            double desiredT = desiredT();
            double distanceToCurve = distance(desiredT);
            double clippedDistance = Range.clip(distanceToCurve, 0.0, correctionDistance);

            Vector tangent = tangentVector(desiredT);

            drive = robotPosition.slope(new Point(
                    bezierX(desiredT) + tangent.x * (correctionDistance - clippedDistance),
                    bezierY(desiredT) + tangent.y * (correctionDistance - clippedDistance)));
        }

        boolean isFinished;
        if(error >= SPLINE_ERROR) {
            drive.scaleMagnitude(SPLINE_P * error);
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