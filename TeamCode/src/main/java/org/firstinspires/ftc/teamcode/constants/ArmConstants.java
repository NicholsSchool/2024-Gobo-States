package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants for the Arm
 */
public interface ArmConstants {
    /** Plane Launcher Servo Minimum Position */
    double PLANE_MIN = 0.35;

    /** Plane Launcher Servo Maximum Position */
    double PLANE_MAX = 1.0;

    /** Maximum Shoulder Motor Power */
    double ARM_MAX = 0.4;

    /** Shoulder Proportional Constant */
    double ARM_P = 0.00075;

    /** Shoulder Vertical Constant */
    double ARM_V = 0.05;

    /** Arm Vertical Encoder Position */
    double ARM_VERTICAL = 2576.0;

    /** Maximum Wrist Motor Power */
    double WRIST_MAX = 0.25;

    /** Wrist Proportional Constant */
    double WRIST_P = 0.001;

    /** Wrist intaking virtual fourbar position */
    double WRIST_INTAKING = 110.0;

    /** Wrist scoring virtual fourbar position */
    double WRIST_SCORING = 1450.0;

    /** Arm position to switch the wrist virtual fourbar position at */
    double FOURBAR_SWITCHING_POSITION = 300;

    /** Arm target position for ground pickup */
    double GROUND_POSITION = 0.0;

    /** Arm target position for drone launch (actual position is 920) */
    double DRONE_POSITION = 1400.0;

    /** Arm target position for scoring pixels (actual position is 920) */
    double SCORING_POSITION = 1100.0;

    /** Arm target position for climbing (actual position is 2100) */
    double CLIMB_POSITION = 2200.0;
}