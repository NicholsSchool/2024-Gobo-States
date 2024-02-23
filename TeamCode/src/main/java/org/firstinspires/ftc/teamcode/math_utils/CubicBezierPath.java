package org.firstinspires.ftc.teamcode.math_utils;

/**
 * Cubic Bezier Curves for BezierPathPlanning
 */
public class CubicBezierPath {
    /** The starting point */
    public Point one;

    /** The first control handle */
    public Point two;

    /** The second control handle */
    public Point three;

    /** The ending point */
    public Point four;

    /**
     * Instantiates the CubicBezierPath
     *
     * @param one the starting point
     * @param two the first control point
     * @param three the second control point
     * @param four the ending point
     */
    public CubicBezierPath(Point one, Point two, Point three, Point four) {
        this.one = one;
        this.two = two;
        this.three = three;
        this.four = four;
    }
}
