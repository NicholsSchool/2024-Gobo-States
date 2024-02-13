package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

/**
 * The Robot Lights.
 * A full list of REV Blinkin patterns can be found
 * <a href="https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html">here</a>.
 */
public class Lights {
    private final RevBlinkinLedDriver leftBlinkin;
    private final RevBlinkinLedDriver rightBlinkin;
    private final BlinkinPattern defaultPattern;

    /**
     * Initializes the Lights
     *
     * @param hwMap the hardwareMap
     * @param isBlueAlliance whether we are blue alliance
     */
    public Lights(HardwareMap hwMap, boolean isBlueAlliance) {
        leftBlinkin = hwMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        rightBlinkin = hwMap.get(RevBlinkinLedDriver.class, "rightBlinkin");

        defaultPattern = isBlueAlliance ? BlinkinPattern.BLUE : BlinkinPattern.RED;
        setAllianceColor();
    }

    /**
     * Sets both left and right LED strips to a certain color pattern.
     *
     * @param pattern The BlinkinPattern to set the LEDs to
     */
    public void setPattern(BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
        rightBlinkin.setPattern(pattern);
    }

    /**
     * Sets the lights to Alliance Colors
     */
    public void setAllianceColor() {
        leftBlinkin.setPattern(defaultPattern);
        rightBlinkin.setPattern(defaultPattern);
    }
}