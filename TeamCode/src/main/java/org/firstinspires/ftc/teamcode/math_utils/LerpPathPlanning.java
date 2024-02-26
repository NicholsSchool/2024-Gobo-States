package org.firstinspires.ftc.teamcode.math_utils;

import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Math for Lerp Path Planning
 */
public class LerpPathPlanning implements SplineConstants {
    private final Drivetrain drivetrain;
    private Point robotPosition;
    private final Point waypoint;
    private final double slope;
    private double c1;
    private double c2;
    private double v2;
    private double v1;


    /**
     * Instantiates the LerpPathPlanning
     *
     * @param drivetrain the drivetrain
     * @param waypoint desired endpoint
     * @param angle angle to be tangent with at the end
     */
    public LerpPathPlanning(Drivetrain drivetrain, Point waypoint, double angle) {
        this.drivetrain = drivetrain;
        this.waypoint = waypoint;

        double angleMod = Math.abs(angle) == Angles.PI_OVER_TWO ? 0.0001 : 0;

        slope = Math.tan(angle + angleMod);

        Point slopePoint = new Point(
                waypoint.x + Math.cos(angle + angleMod),
                waypoint.y + Math.sin(angle + angleMod));

        c1 = waypoint.x;
        v1 = waypoint.y;
        c2 = slopePoint.x;
        v2 = slopePoint.y;
    }

    // used to make the line the bot is projected on, tangent to end angle
    private double lineProjection(double x){
        return slope * (x - waypoint.x) + waypoint.y;
    }

    //closest x value
    private double optimalX(){
        return (c2 * Math.pow(v1, 2) + (robotPosition.y * (c1 - c2) - (c1 + c2) * v2) * v1 - robotPosition.y * (c1 - c2) * v2 + robotPosition.x * Math.pow(c1 - c2, 2)
                + c1 * Math.pow(v2 , 2)) / (Math.pow(v1, 2) - 2 * v2 * v1 + Math.pow(c1 , 2) - 2 * c1 * c2 + Math.pow(c2, 2) + Math.pow(v2 , 2));
    }

    //distance from optimal x on the line to waypoint
    private double distanceOnLine(){
        return Math.sqrt(Math.pow(c1 - optimalX(), 2) + Math.pow(v1 - lineProjection(optimalX()), 2));
    }

    //how far the bot is from the tangent line
    private double fromLine(){
        return Math.hypot(robotPosition.x - optimalX(), robotPosition.y - lineProjection(optimalX()));
    }


    //desiredT for point it's correcting to
    private double desiredT(){
        return 1 - fromLine() / distanceOnLine();
    }

    //distance from projected point on the line to the waypoint
    private double projectedDistance(){
        return Math.sqrt(Math.pow(desiredT() * (c1 - optimalX()) + optimalX() - c1, 2)
                + Math.pow(desiredT() * (v1 - lineProjection(optimalX())) + lineProjection(optimalX()) - v1, 2));
    }

    /**
     * With the robot at (x, y), calculates the drive vector of the robot
     * in order to reach the waypoint
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

        double error = robotPosition.distance(waypoint);
        double vx;
        double vy;

        //makes sure it doesn't move away from the point, goes normal in this case
        if(distanceOnLine() > projectedDistance()) {
            vx = desiredT() * (c1 - optimalX()) + optimalX() - robotPosition.x;
            vy = desiredT() * (v1 - lineProjection(optimalX())) + lineProjection(optimalX()) - robotPosition.y;
        }
        else {
            vx = optimalX() - robotPosition.x;
            vy = lineProjection(optimalX()) - robotPosition.y;
        }
        Vector driveVector =  new Vector(vx, vy);

        boolean isFinished;
        if(error >= SPLINE_ERROR) {
            driveVector.scaleMagnitude(SPLINE_P * error);
            isFinished = false;
        }
        else {
            driveVector.zero();
            isFinished = true;
        }

        drivetrain.drive(driveVector, turn, autoAlign, lowGear);
        return isFinished;
    }
}