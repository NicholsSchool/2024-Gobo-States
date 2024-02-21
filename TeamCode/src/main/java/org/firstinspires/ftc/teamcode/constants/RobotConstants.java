package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants for the Robot
 */
public interface RobotConstants {
    /** Label for Blue Alliance */
    boolean IS_BLUE_ALLIANCE = true;

    /** Label for Red Alliance */
    boolean IS_RED_ALLIANCE = !IS_BLUE_ALLIANCE;

    /** Label for Audience side start in Autonomous */
    boolean IS_AUDIENCE_SIDE = true;

    /** Label for Backstage side start in Autonomous */
    boolean IS_BACKSTAGE_SIDE = !IS_AUDIENCE_SIDE;
}