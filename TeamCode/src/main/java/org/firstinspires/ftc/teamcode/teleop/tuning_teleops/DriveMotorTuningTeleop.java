package org.firstinspires.ftc.teamcode.teleop.tuning_teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.math_utils.VectorMotionProfile;

/**
 * Teleop for Tuning Drive Motors
 */
@Config
@TeleOp(name="Drive Motor Tuning", group="Tuning")
public class DriveMotorTuningTeleop extends OpMode implements DrivetrainConstants {
    private ElapsedTime loopTime;
    private FtcDashboard dashboard;
    private Controller controller;
    private DcMotorEx leftDrive, rightDrive, backDrive;
    private VectorMotionProfile driveProfile;
    private MotionProfile turnProfile;

    public static double p;
    public static double i;
    public static double d;
    public static double backFF;
    public static double leftFF;
    public static double rightFF;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        controller = new Controller(gamepad1);

        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        backDrive = hardwareMap.get(DcMotorEx.class, "backDrive");

        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backDrive.setDirection(DcMotorEx.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
        rightDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);
        backDrive.setVelocityPIDFCoefficients(DRIVE_P, DRIVE_I, 0.0, 0.0);

        driveProfile = new VectorMotionProfile(DRIVE_PROFILE_SPEED * .5);
        turnProfile = new MotionProfile(TURN_PROFILE_SPEED * .5, TURN_PROFILE_MAX);

        loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        controller.update();

        leftDrive.setVelocityPIDFCoefficients(p, i, d, leftFF);
        rightDrive.setVelocityPIDFCoefficients(p, i, d, rightFF);
        backDrive.setVelocityPIDFCoefficients(p, i, d, backFF);

        double[] targets = this.drive(
                controller.leftStick.toVector(),
                controller.rightStick.x.value(),
                controller.leftTrigger.value() <= 0.0);

        telemetry.addData("left velocity", leftDrive.getVelocity());
        telemetry.addData("right velocity", rightDrive.getVelocity());
        telemetry.addData("back velocity", backDrive.getVelocity());
        telemetry.addData("left target", targets[0]);
        telemetry.addData("right target", targets[1]);
        telemetry.addData("back target", targets[2]);

        telemetry.addData("loop time", loopTime.time());
        loopTime.reset();
        telemetry.update();
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    /**
     * Drives the robot field oriented
     *
     * @param driveInput the (x, y) input
     * @param turn the turning input
     * @param lowGear whether to put the robot to virtual low gear
     *
     * @return the left, right, and back target velocities
     */
    public double[] drive(Vector driveInput, double turn, boolean lowGear) {
        turn = turnProfile.calculate(turn);

        driveInput = driveProfile.calculate(driveInput.clipMagnitude(
                (lowGear ? VIRTUAL_LOW_GEAR : VIRTUAL_HIGH_GEAR) - Math.abs(turn)));
        double power = driveInput.magnitude();
        double angle = driveInput.angle();

        double left = (turn + power *
                Math.cos(angle + LEFT_DRIVE_OFFSET - Angles.PI_OVER_TWO)) * MAX_MOTOR_VELOCITY;
        double right = (turn + power *
                Math.cos(angle + RIGHT_DRIVE_OFFSET - Angles.PI_OVER_TWO)) * MAX_MOTOR_VELOCITY;
        double back = (turn + power *
                Math.cos(angle + BACK_DRIVE_OFFSET - Angles.PI_OVER_TWO)) * MAX_MOTOR_VELOCITY;

        leftDrive.setVelocity(left);
        rightDrive.setVelocity(right);
        backDrive.setVelocity(back);

        return new double[]{left, right, back};
    }
}