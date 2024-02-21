package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants for Spline
 */
public interface SplineConstants {
    /** Auto Driving Proportional Constant */
    double SPLINE_P = 0.05;

    /** Auto Driving allowed error */
    double SPLINE_ERROR = 0.5;

    /** The correction distance for Bezier */
    double CORRECTION_DISTANCE = 20.0;

    /** the number of steps for Bezier */
    int STEPS = 200;
}