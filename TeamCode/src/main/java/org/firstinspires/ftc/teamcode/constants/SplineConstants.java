package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants for Spline
 */
public interface SplineConstants {
    /** Auto Driving Proportional Constant */
    double SPLINE_P = 0.05;

    /** Auto Driving allowed error */
    double SPLINE_ERROR = 0.5;

    /** Spline Left Waypoint Coordinate X Value */
    double LEFT_WAYPOINT_X = -14.0;

    /** Spline Right Waypoint Coordinate X Value */
    double RIGHT_WAYPOINT_X = 38.0;

    /** Spline Intake Coordinate X Value */
    double INTAKE_X = 58.0;

    /** Spline Scoring Coordinate X Value */
    double SCORING_X = -48.0;

    /** The X value for launching the plane */
    double PLANE_LAUNCHING_X = -24.0;

    /** Spline Intake Coordinate Blue Y Value */
    double BLUE_INTAKE_Y = 56.0;

    /** Spline Intake Coordinate Red Y Value */
    double RED_INTAKE_Y = -56.0;

    /** Spline Waypoint Coordinate Blue Y Value */
    double BLUE_WAYPOINT_Y = -36.0;

    /** Spline Waypoint Coordinate Red Y Value */
    double RED_WAYPOINT_Y = 36.0;

    /** Spline Waypoint Coordinate Y Value when going to scoring */
    double CENTER_WAYPOINT_Y = 0.0;

    /** The Closest to Driver Scoring Y For Blue Alliance */
    double BLUE_SCORING_Y_CLOSE = -42.0;

    /** The Middle Distance to Driver Scoring Y For Blue Alliance */
    double BLUE_SCORING_Y_MID = -36.0;

    /** The Farthest to Driver Scoring Y For Blue Alliance */
    double BLUE_SCORING_Y_FAR = -30.0;

    /** The Farthest to Driver Scoring Y For Red Alliance */
    double RED_SCORING_Y_FAR = 30.0;

    /** The Middle Distance to Driver Scoring Y For Red Alliance */
    double RED_SCORING_Y_MID = 36.0;

    /** The Closest to Driver Scoring Y For Red Alliance */
    double RED_SCORING_Y_CLOSE = 42.0;
}