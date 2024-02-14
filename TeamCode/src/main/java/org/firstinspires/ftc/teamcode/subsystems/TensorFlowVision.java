package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Robot Tensor Flow Vision
 */
public class TensorFlowVision implements VisionConstants {
    /**
     * The Three Autonomous Prop Locations
     */
    public enum PropLocation {
        LEFT, CENTER, RIGHT
    }

    private final TfodProcessor processor;
    private final VisionPortal portal;

    /**
     * Initializes the TensorFlowVision
     *
     * @param hwMap the hardware map
     */
    public TensorFlowVision(HardwareMap hwMap) {
        processor = new TfodProcessor.Builder()
                .setModelAssetName("uniPropV1.tflite")
                .setModelLabels(new String[]{"blue face", "red face"})
                .setIsModelTensorFlow2(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(processor)
                .enableLiveView(true)
                .build();
    }

    /**
     * Gets the Prop Location using the best current Prop Recognition
     *
     * @return the Enum representing the Prop Location
     */
    public PropLocation getPropLocation() {
        List<Recognition> currentRecognitions = processor.getRecognitions();

        float bestConfidence = 0;
        Recognition bestRecognition = null;

        for (Recognition currentRecognition : currentRecognitions) {
            if(currentRecognition.getConfidence() > bestConfidence) {
                bestRecognition = currentRecognition;
                bestConfidence = currentRecognition.getConfidence();
            }
        }

        if(bestRecognition == null)
            return null;

        double propPosition = (bestRecognition.getLeft() + bestRecognition.getRight() ) * 0.5;

        if(propPosition < LEFT_PROP_THRESHOLD)
            return PropLocation.LEFT;
        return propPosition < RIGHT_PROP_THRESHOLD ? PropLocation.CENTER : PropLocation.RIGHT;
    }

    /**
     * Closes the Vision Portal
     */
    public void close() {
        portal.close();
    }
}