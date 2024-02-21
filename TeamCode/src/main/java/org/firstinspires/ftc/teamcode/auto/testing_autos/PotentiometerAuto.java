package org.firstinspires.ftc.teamcode.auto.testing_autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Potentiometer Testing
 */
@Autonomous(name="Potentiometer Testing", group="Testing")
public class PotentiometerAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput pot = hardwareMap.get(AnalogInput.class, "pot");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("pot current", pot.getVoltage());
            telemetry.addData("pot max", pot.getMaxVoltage());
            telemetry.update();
        }
    }
}