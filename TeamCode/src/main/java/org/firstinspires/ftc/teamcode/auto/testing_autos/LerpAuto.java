package org.firstinspires.ftc.teamcode.auto.testing_autos;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.LerpPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

/**
 * Testing Auto for Lerp
 */
@Autonomous(name="Lerp Testing", group="Testing")
public class LerpAuto extends LinearOpMode implements RobotConstants {

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, 0, 0, Angles.PI_OVER_TWO, IS_BLUE_ALLIANCE);
        drivetrain.setFloat();
        LerpPathPlanning spline = new LerpPathPlanning(drivetrain, new Point(50, 30), Angles.PI_OVER_TWO);
        Lights lights = new Lights(hardwareMap, IS_BLUE_ALLIANCE);

        waitForStart();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);

        boolean isFinished = false;
        while(opModeIsActive() && !isFinished) {
            isFinished = spline.spline(0.0, true, true);
        }
    }
}