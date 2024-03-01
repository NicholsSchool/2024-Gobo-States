package org.firstinspires.ftc.teamcode.auto.competition_autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;

/**
 * Red Backstage Side Competition Autonomous
 */
@Autonomous(name="Red Backstage", group="Competition")
public class RedBackstageAuto extends LinearOpMode implements RobotConstants {
    @Override
    public void runOpMode() {
        AutonomousRobot robot = new AutonomousRobot(hardwareMap, -12.0, 63.0,
                Angles.NEGATIVE_PI_OVER_TWO, IS_RED_ALLIANCE, IS_BACKSTAGE_SIDE, telemetry);

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