package org.firstinspires.ftc.teamcode.constants;

/**
 * Constants for the Robot
 */
public interface RobotConstants {
    /** Label for Blue Alliance in robot constructors */
    boolean IS_BLUE_ALLIANCE = true;

    /** Label for Red Alliance in robot constructors */
    boolean IS_RED_ALLIANCE = !IS_BLUE_ALLIANCE;

    /** Label for Audience side start in Autonomous */
    boolean IS_AUDIENCE = true;

    /** Label for Backstage side start in Autonomous */
    boolean IS_BACKSTAGE = !IS_AUDIENCE;
}