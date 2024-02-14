package org.firstinspires.ftc.teamcode.auto.competition_autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;

/**
 * Red Audience Side Competition Autonomous
 */
@Autonomous(name="Red Audience", group="Competition")
public class RedAudienceAuto extends LinearOpMode implements RobotConstants {
    @Override
    public void runOpMode() {
        AutonomousRobot robot = new AutonomousRobot(hardwareMap, 36.0, 65.0,
                Angles.NEGATIVE_PI_OVER_TWO, IS_RED_ALLIANCE, IS_AUDIENCE, telemetry);

        while(opModeInInit())
            robot.updatePropLocation();

        waitForStart();
    }
}