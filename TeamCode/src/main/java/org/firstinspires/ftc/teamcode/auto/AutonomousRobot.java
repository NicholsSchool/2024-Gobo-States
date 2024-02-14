package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.PotConstants;
import org.firstinspires.ftc.teamcode.math_utils.ParabolicPathPlanning;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.TensorFlowVision;
import org.firstinspires.ftc.teamcode.subsystems.TensorFlowVision.PropLocation;

/**
 * Robot Container for the Autonomous Period
 */
public class AutonomousRobot implements PotConstants, ArmConstants {
    /**
     * The two target columns for autonomous
     */
    public enum TargetColumn {
        LEFT, RIGHT
    }

    private final ElapsedTime timer;
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Hand hand;
    private final Lights lights;
    private final TensorFlowVision tensorFlowVision;
    private AprilTagVision aprilTagVision;
    private ParabolicPathPlanning parabolicPathPlanning;
    private final AnalogInput pot;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final TargetColumn targetColumn;
    private PropLocation propLocation;

    /**
     * @param hardwareMap the hardware map
     * @param x the initial x
     * @param y the initial y
     * @param angle the initial angle
     * @param isBlue whether we are blue alliance
     * @param telemetry the telemetry
     */
    public AutonomousRobot(HardwareMap hardwareMap,
                           double x, double y, double angle, boolean isBlue, Telemetry telemetry) {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        arm = new Arm(hardwareMap, ARM_AUTO_OFFSET, WRIST_AUTO_OFFSET);
        drivetrain = new Drivetrain(hardwareMap, x, y, angle);
        hand = new Hand(hardwareMap);
        lights = new Lights(hardwareMap, isBlue);
        tensorFlowVision = new TensorFlowVision(hardwareMap);
        parabolicPathPlanning = new ParabolicPathPlanning(drivetrain, isBlue);
        pot = hardwareMap.get(AnalogInput.class, "pot");
        targetColumn = pot.getVoltage() >= THRESHOLD ? TargetColumn.LEFT : TargetColumn.RIGHT;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /**
     * Updates the Prop Position using the current best recognition.
     * Call looped during init()
     */
    public void updatePropLocation() {
        PropLocation currentLocation = tensorFlowVision.getPropLocation();
        if(currentLocation != null)
            propLocation = currentLocation;
        telemetry.addData("Prop Location", propLocation);
    }

    /**
     * Finalizes the Prop Location to Center if null,
     * Closes TensorFlowVision and Initializes AprilTagVision
     */
    public void transitionVision() {
        tensorFlowVision.close();
        aprilTagVision = new AprilTagVision(hardwareMap);

        if(propLocation == null)
            propLocation = PropLocation.CENTER;
    }
}