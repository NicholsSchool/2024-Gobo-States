package org.firstinspires.ftc.teamcode.auto.competition_autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;

/**
 * Blue Audience Side Competition Autonomous
 */
@Autonomous(name="Blue Audience", group="Competition")
public class BlueAudienceAuto extends LinearOpMode implements RobotConstants {
    @Override
    public void runOpMode() {
        AutonomousRobot robot = new AutonomousRobot(hardwareMap, 36.0, -65.0,
                Angles.PI_OVER_TWO, IS_BLUE_ALLIANCE, IS_AUDIENCE, telemetry);

        while(opModeInInit())
            robot.updatePropLocation();

        waitForStart();

        robot.closeVision();
    }
}