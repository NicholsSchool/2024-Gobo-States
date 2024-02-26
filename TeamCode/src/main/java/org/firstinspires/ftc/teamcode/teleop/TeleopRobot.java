package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.RobotPose;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

/**
 * Robot Container for the Teleop Period
 */
public class TeleopRobot implements ArmConstants, DrivetrainConstants {
    private final Controller driverOI;
    private final Controller operatorOI;
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Hand hand;
    private final Lights lights;
    private final double[] armSetPositions;
    private final double[] autoAlignPositions;
    private final double directionCoefficient;
    private boolean resetFieldOriented;

    /**
     * Instantiates the TeleopRobot
     *
     * @param hwMap the hardwareMap
     * @param x the initial X value
     * @param y the initial Y value
     * @param angle the initial
     * @param isBlue whether we are blue alliance
     * @param g1 gamepad 1
     * @param g2 gamepad 2
     */
    public TeleopRobot(HardwareMap hwMap,
                       double x, double y, double angle, boolean isBlue, Gamepad g1, Gamepad g2) {
        arm = new Arm(hwMap, 0, 0);
        drivetrain = new Drivetrain(hwMap, x, y, angle, isBlue);
        hand = new Hand(hwMap);
        lights = new Lights(hwMap, isBlue);
        driverOI = new Controller(g1);
        operatorOI = new Controller(g2);

        armSetPositions = new double[]{
                GROUND_POSITION, SCORING_POSITION, DRONE_POSITION, CLIMB_POSITION};

        autoAlignPositions = isBlue ?
                new double[]{Angles.PI_OVER_TWO, Angles.NEGATIVE_PI_OVER_TWO, 0.0, -Math.PI} :
                new double[]{Angles.NEGATIVE_PI_OVER_TWO, Angles.PI_OVER_TWO, -Math.PI, 0.0};

        directionCoefficient = isBlue ? 1.0 : -1.0;
    }

    /**
     * Updates all TeleopRobot instances
     */
    public void update() {
        updateSubsystems();
        armControls();
        drivetrainControls();
        handControls();
        lightsControls();
    }

    private void updateSubsystems() {
        driverOI.update();
        operatorOI.update();

        drivetrain.updateHeadingOnly();
    }

    private void armControls() {
        if(operatorOI.options.wasJustPressed())
            arm.offsetEncoders();

        if(operatorOI.share.wasJustPressed())
            arm.togglePlane();

        if(operatorOI.dpadDown.wasJustPressed())
            arm.setTargetArmPosition(armSetPositions[0]);
        else if(operatorOI.dpadLeft.wasJustPressed())
            arm.setTargetArmPosition(armSetPositions[1]);
        else if(operatorOI.dpadRight.wasJustPressed())
            arm.setTargetArmPosition(armSetPositions[2]);
        else if(operatorOI.dpadUp.wasJustPressed())
            arm.setTargetArmPosition(armSetPositions[3]);

        if(operatorOI.leftTrigger.value() >= 0.5 && operatorOI.rightTrigger.value() >= 0.5) {
            arm.climb(operatorOI.leftStick.y.value());
            arm.setTargetArmPosition(arm.getArmPosition());
        }
        else if(operatorOI.leftStick.y.hasBeenZero())
            arm.armToPosition();
        else {
            arm.armManual(operatorOI.leftStick.y.value());
            arm.setTargetArmPosition(arm.getArmPosition());
        }

        if(operatorOI.x.wasJustPressed())
            arm.setTargetWristPosition(-50.0);

        if(operatorOI.triangle.wasJustPressed())
            arm.setTargetWristPosition(1400);

        if(operatorOI.x.isPressed() || operatorOI.triangle.isPressed())
            arm.wristToPosition();
        else if(operatorOI.rightStick.y.hasBeenZero() &&
                !operatorOI.square.isPressed() &&
                !operatorOI.circle.isPressed())
            arm.virtualFourbar();
        else
            arm.wristManual(operatorOI.rightStick.y.value());
    }

    private void drivetrainControls() {
        if(driverOI.share.wasJustPressed()) {
            resetFieldOriented = true;
            drivetrain.resetFieldOriented();
        }
        else
            resetFieldOriented = false;

        Vector driveVector;
        if(driverOI.dpadDown.isPressed())
            driveVector = new Vector(0.0, -VIRTUAL_LOW_GEAR);
        else if(driverOI.dpadLeft.isPressed())
            driveVector = new Vector(-VIRTUAL_LOW_GEAR, 0.0);
        else if(driverOI.dpadRight.isPressed())
            driveVector = new Vector(VIRTUAL_LOW_GEAR, 0.0);
        else if(driverOI.dpadUp.isPressed())
            driveVector = new Vector(0.0, VIRTUAL_LOW_GEAR);
        else
            driveVector = driverOI.leftStick.toVector();

        double turn = driverOI.rightStick.x.value();

        boolean lowGear = driverOI.leftTrigger.value() <= 0.5;

        driveVector.x *= directionCoefficient;
        driveVector.y *= directionCoefficient;

        if(lowGear) {
            driveVector.x *= VIRTUAL_LOW_GEAR;
            driveVector.y *= VIRTUAL_LOW_GEAR;
        }
        else {
            driveVector.x *= VIRTUAL_HIGH_GEAR;
            driveVector.y *= VIRTUAL_HIGH_GEAR;
        }

        boolean autoAlign = driverOI.rightStick.x.hasBeenZero();

        if(!autoAlign)
            drivetrain.setTargetHeading(drivetrain.getRobotPose().angle);
        else if(driverOI.triangle.wasJustPressed())
            drivetrain.setTargetHeading(autoAlignPositions[0]);
        else if(driverOI.x.wasJustPressed())
            drivetrain.setTargetHeading(autoAlignPositions[1]);
        else if(driverOI.circle.wasJustPressed())
            drivetrain.setTargetHeading(autoAlignPositions[2]);
        else if(driverOI.square.wasJustPressed())
            drivetrain.setTargetHeading(autoAlignPositions[3]);

        drivetrain.drive(driveVector, turn, autoAlign, lowGear);
    }

    private void handControls() {
        if(operatorOI.leftBumper.wasJustPressed())
            hand.toggleLeft();
        if(operatorOI.rightBumper.wasJustPressed())
            hand.toggleRight();
    }

    private void lightsControls() {
        if(resetFieldOriented)
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else
            lights.setAllianceColor();
    }
}