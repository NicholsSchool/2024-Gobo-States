package org.firstinspires.ftc.teamcode.auto.demo_autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.ParabolicPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Path Planning Demo
 */
@Autonomous(name="Path Planning Demo", group="Demo")
public class PathPlanningDemoAuto extends LinearOpMode implements RobotConstants, SplineConstants {
    @Override
    public void runOpMode() {
        Drivetrain drivetrain;
        ParabolicPathPlanning parabolicPathPlanning;

        waitForStart();
        drivetrain = new Drivetrain(hardwareMap, 0.0, 0.0, Angles.PI_OVER_TWO, IS_BLUE_ALLIANCE);
        drivetrain.setFloat();

        parabolicPathPlanning = new ParabolicPathPlanning();
        Point destination = new Point(72.0, 48.0);

        boolean isFinished = false;
        while(opModeIsActive() && !isFinished) {
            drivetrain.update();
            Point robot = drivetrain.getRobotPose().toPoint();
            double distance = robot.distance(destination);

            Vector driveVector = parabolicPathPlanning.vectorToVertex(
                    robot, destination, true);

            if(distance >= DESTINATION_ERROR)
                driveVector.scaleMagnitude(SPLINE_P * distance);
            else
                driveVector.zero();

            drivetrain.drive(driveVector, 0.0, true, true);
        }
    }
}