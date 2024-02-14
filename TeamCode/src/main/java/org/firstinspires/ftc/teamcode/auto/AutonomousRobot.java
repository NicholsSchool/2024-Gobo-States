package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.constants.PotConstants;
import org.firstinspires.ftc.teamcode.math_utils.ParabolicPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
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
     * @param isAudience whether we are on the audience side
     * @param telemetry the telemetry
     */
    public AutonomousRobot(HardwareMap hardwareMap, double x, double y, double angle,
                           boolean isBlue, boolean isAudience, Telemetry telemetry) {

        arm = new Arm(hardwareMap, ARM_AUTO_OFFSET, WRIST_AUTO_OFFSET);
        drivetrain = new Drivetrain(hardwareMap, x, y, angle);
        parabolicPathPlanning = new ParabolicPathPlanning(drivetrain, isBlue, isAudience);
        hand = new Hand(hardwareMap);
        lights = new Lights(hardwareMap, isBlue);
        tensorFlowVision = new TensorFlowVision(hardwareMap);

        pot = hardwareMap.get(AnalogInput.class, "pot");
        targetColumn = pot.getVoltage() >= THRESHOLD ? TargetColumn.LEFT : TargetColumn.RIGHT;

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /**
     * Updates the Prop Position using the current best recognition.
     * Call within a loop during init()
     */
    public void updatePropLocation() {
        PropLocation currentLocation = tensorFlowVision.getPropLocation();
        if(currentLocation != null)
            propLocation = currentLocation;
        telemetry.addData("Prop Location", propLocation);
    }

    private void transitionVision() {
        tensorFlowVision.close();
        aprilTagVision = new AprilTagVision(hardwareMap);

        if(propLocation == null)
            propLocation = PropLocation.CENTER;
    }

    private boolean purpleLoopSequence() {
        arm.armToPosition();
        arm.virtualFourbar();
        lights.setAllianceColor();
         return parabolicPathPlanning.splineToPurple(0.0, true, true);
    }

    /**
     * Runs the full Autonomous Routine
     */
    public void runAutonomousRoutine() {
        transitionVision();

        arm.setTargetArmPosition(SCORING_POSITION);
        drivetrain.setTargetHeading(0.0);

        boolean isFinished = false;
        while(!isFinished)
            isFinished = purpleLoopSequence();
    }
}