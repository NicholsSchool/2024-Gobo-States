package org.firstinspires.ftc.teamcode.math_utils;

/**
 * A Lerp Path is defined by a Waypoint and a control point
 */
public class LerpPath {
    /**
     * The target waypoint
     */
    public final Point waypoint;

    /**
     * The angle to arrive at the waypoint at in radians
     */
    public double angle;

    /**
     * The control point defined by the slope and waypoint
     */
    public Point slopePoint;

    /**
     * The slope of the line formed by the waypoint and slope point
     */
    public double slope;

    /**
     * Instantiates the LerpPath
     *
     * @param waypoint the target waypoint
     * @param angle the target arrival angle
     */
    public LerpPath(Point waypoint, double angle) {
        this.waypoint = waypoint;

        if(Math.abs(angle) == Angles.PI_OVER_TWO)
            angle += 0.00001;

        slope = Math.tan(angle);

        slopePoint = new Point(
                waypoint.x + Math.cos(angle),
                waypoint.y + Math.sin(angle));

        this.angle = angle;
    }
}
