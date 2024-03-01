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
        AutonomousRobot robot = new AutonomousRobot(hardwareMap, 36.0, -63.0,
                Angles.PI_OVER_TWO, IS_BLUE_ALLIANCE, IS_AUDIENCE_SIDE, telemetry);

        while(opModeInInit())
            robot.updatePropLocation();

        waitForStart();

        robot.closeVision();

        robot.prepForPathOne();
        boolean pathOneIsFinished = false;
        while(opModeIsActive() && !pathOneIsFinished)
            pathOneIsFinished = robot.followPathOne();

        robot.prepForPurplePixelDrop();
        boolean purpleFinished = false;
        while(opModeIsActive() && !purpleFinished)
            purpleFinished = robot.dropPurplePixel();

        boolean waitDone = false;
        while(opModeIsActive() && !waitDone)
            waitDone = robot.waitAfterPurple();

        robot.prepForPathTwo();
        boolean pathTwoIsFinished = false;
        while(opModeIsActive() && !pathTwoIsFinished)
            pathTwoIsFinished = robot.followPathTwo();

        robot.prepForPathThree();
        boolean pathThreeIsFinished = false;
        while(opModeIsActive() && !pathThreeIsFinished)
            pathThreeIsFinished = robot.followPathThree();

        robot.prepForPathFour();
        boolean pathFourIsFinished = false;
        while(opModeIsActive() && !pathFourIsFinished)
            pathFourIsFinished = robot.followPathFour();

        robot.prepForPathFive();
        boolean pathFiveIsFinished = false;
        while(opModeIsActive() && !pathFiveIsFinished)
            pathFiveIsFinished = robot.followPathFive();

        robot.prepForPathSix();
        boolean pathSixIsFinished = false;
        while(opModeIsActive() && !pathSixIsFinished)
            pathSixIsFinished = robot.followPathSix();

        robot.prepForNudge();
        boolean nudgeDone = false;
        while(opModeIsActive() && !nudgeDone)
            nudgeDone = robot.nudgeForward();

        robot.prepForYellowPixelPlace();
        boolean yellowPixelPlaced = false;
        while(opModeIsActive() && !yellowPixelPlaced)
            yellowPixelPlaced = robot.placeYellowPixel();
    }
}