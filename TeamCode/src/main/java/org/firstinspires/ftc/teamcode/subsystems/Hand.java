package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HandConstants;

/**
 * The Robot Hand
 */
public class Hand implements HandConstants {
    private final Servo leftGrabber;
    private final Servo rightGrabber;
    private boolean leftPosition;
    private boolean rightPosition;

    /**
     * Initializes the Hand
     */
    public Hand(HardwareMap hardwareMap) {
        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        leftGrabber.scaleRange(HandConstants.LEFT_OUT, LEFT_IN);
        leftGrabber.setPosition(1.0);

        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");
        rightGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber.scaleRange(HandConstants.RIGHT_IN, RIGHT_OUT);
        rightGrabber.setPosition(0.0);

        leftPosition = true;
        rightPosition = true;
    }

    /**
     * Toggles the left grabber
     * */
    public void toggleLeft() {
        leftGrabber.setPosition(leftPosition ? 0.0 : 1.0);
        leftPosition = ! leftPosition;
    }

    /**
     * Toggles the right grabber
     * */
    public void toggleRight() {
        rightGrabber.setPosition(rightPosition ? 1.0 : 0.0);
        rightPosition = !rightPosition;
    }
}