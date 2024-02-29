package org.firstinspires.ftc.teamcode.auto.competition_autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.AutonomousRobot;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;

/**
 * Blue Backstage Side Competition Autonomous
 */
@Autonomous(name="Blue Backstage", group="Competition")
public class BlueBackstageAuto extends LinearOpMode implements RobotConstants {
    @Override
    public void runOpMode() {
        AutonomousRobot robot = new AutonomousRobot(hardwareMap, -12.0, -63.0,
                Angles.PI_OVER_TWO, IS_BLUE_ALLIANCE, IS_BACKSTAGE_SIDE, telemetry);

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
            waitDone = robot.wait(1.0);

        robot.prepForPathTwo();
        boolean pathTwoIsFinished = false;
        while(opModeIsActive() && !pathTwoIsFinished)
            pathTwoIsFinished = robot.followPathTwo();

        robot.prepForPathThree();
        boolean pathThreeIsFinished = false;
        while(opModeIsActive() && !pathThreeIsFinished)
            pathThreeIsFinished = robot.followPathThree();

        robot.prepForYellowPixelPlace();
        boolean yellowPixelPlaced = false;
        while(opModeIsActive() && !yellowPixelPlaced)
            yellowPixelPlaced = robot.placeYellowPixel();
    }
}