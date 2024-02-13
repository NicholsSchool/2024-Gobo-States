package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.math_utils.FeedbackController;
import org.firstinspires.ftc.teamcode.math_utils.SimpleFeedbackController;

/**
 * Robot Arm
 */
public class Arm implements ArmConstants {
    private final DcMotorEx leftShoulder;
    private final DcMotorEx rightShoulder;
    private final DcMotorEx wrist;
    private final Servo planeLauncher;
    private final FeedbackController armController;
    private final SimpleFeedbackController wristController;
    private double wristTargetPosition;
    private int armOffset;
    private int wristOffset;
    private boolean launcherPosition;

    /**
     * Initializes the Arm
     *
     * @param hardwareMap the hardware map
     * @param armOffset the initial arm offset
     * @param wristOffset the initial wrist offset
     */
    public Arm(HardwareMap hardwareMap, int armOffset, int wristOffset) {
        leftShoulder = hardwareMap.get(DcMotorEx.class, "leftShoulder");
        leftShoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShoulder.setDirection(DcMotorEx.Direction.REVERSE);

        rightShoulder = hardwareMap.get(DcMotorEx.class, "rightShoulder");
        rightShoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShoulder.setDirection(DcMotorEx.Direction.REVERSE);

        wrist = hardwareMap.get(DcMotorEx.class, "wrist");
        wrist.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        wrist.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wrist.setDirection(DcMotorEx.Direction.REVERSE);

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        planeLauncher.setDirection(Servo.Direction.FORWARD);
        planeLauncher.scaleRange(ArmConstants.PLANE_MIN, ArmConstants.PLANE_MAX);

        armController = new FeedbackController(ARM_P, 0.0, ARM_V, ARM_VERTICAL);
        wristController = new SimpleFeedbackController(WRIST_P);

        this.armOffset = armOffset;
        this.wristOffset = wristOffset;
    }

    /**
     * Toggles the Plane Launcher
     */
    public void togglePlane() {
        planeLauncher.setPosition(launcherPosition ? 0.0 : 1.0);
        launcherPosition = ! launcherPosition;
    }

    /**
     * Sets arm power without a governor. Only allows downwards motion.
     *
     * @param power the power proportion
     */
    public void climb(double power) {
        armNoGovernor(-Math.abs(power));
    }

    /**
     * Moves both shoulder motors together manually
     *
     * @param power the input motor power
     */
    public void armManual(double power) {
        armNoGovernor(Range.clip(power, -ARM_MAX, ARM_MAX));
    }

    private void armNoGovernor(double power) {
        leftShoulder.setPower(power);
        rightShoulder.setPower(power);
    }

    /**
     * The position of the arm measured in thru bore ticks
     *
     * @return the encoder position of the arm
     */
    public int getArmPosition() {
        return leftShoulder.getCurrentPosition() - armOffset;
    }

    /**
     * Moves the arm to the target position using a feedback loop
     */
    public void armToPosition() {
        armManual(armController.calculate(getArmPosition()));
    }

    /**
     * Sets the arm target position
     *
     * @param targetPosition the target thru bore encoder position
     */
    public void setTargetArmPosition(double targetPosition) {
        armController.setTargetPosition(targetPosition);
    }

    /**
     * Moves the wrist motor manually
     *
     * @param power the input motor power
     */
    public void wristManual(double power) {
        wrist.setPower(Range.clip(power, -WRIST_MAX, WRIST_MAX));
    }

    /**
     * The position of the wrist measured in thru bore ticks
     *
     * @return the encoder position of the wrist
     */
    public int getWristPosition() {
        return wrist.getCurrentPosition() - wristOffset;
    }

    /**
     * Moves the wrist to the target position using a feedback loop
     */
    public void wristToPosition() {
        wristManual(wristController.calculate(wristTargetPosition - getWristPosition()));
    }

    /**
     * Sets the wrist target position
     *
     * @param targetPosition the target thru bore encoder position
     */
    public void setTargetWristPosition(double targetPosition) {
        wristTargetPosition = targetPosition;
    }

    /**
     * Moves the wrist to the intaking or scoring positions automatically
     */
    public void virtualFourbar() {
        double armPosition = getArmPosition();
        if(armPosition <= FOURBAR_SWITCHING_POSITION)
            setTargetWristPosition(WRIST_INTAKING - armPosition);
        else
            setTargetWristPosition(WRIST_SCORING - armPosition);
        wristToPosition();
    }

    /**
     * Sets the arm to Float mode
     */
    public void setArmFloat() {
        leftShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Sets the wrist to Float mode
     */
    public void setWristFloat() {
        wrist.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Offsets the arm and wrist encoders. Use when arm and wrist are fully down
     */
    public void offsetEncoders() {
        armOffset = leftShoulder.getCurrentPosition();
        wristOffset = wrist.getCurrentPosition();
    }
}