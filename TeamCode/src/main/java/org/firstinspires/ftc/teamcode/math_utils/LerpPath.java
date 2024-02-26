package org.firstinspires.ftc.teamcode.math_utils;

/**
 * A Lerp Path is defined by a Waypoint and a control point
 */
public class LerpPath {
    public final Point waypoint;
    public final Point controlPoint;

    /**
     * Instantiates the LerpPath
     *
     * @param waypoint the target waypoint
     * @param controlPoint the control point
     */
    public LerpPath(Point waypoint, Point controlPoint) {
        this.waypoint = waypoint;
        this.controlPoint = controlPoint;
    }
}
