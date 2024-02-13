package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.math_utils.RobotPose;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Robot April Tag Vision
 */
public class AprilTagVision implements VisionConstants {
    private final AprilTagProcessor processor;
    private double sumOfWeights;
    private int usefulDetections;

    /**
     * Instantiates the AprilTagVision
     *
     * @param hwMap the hardware map
     */
    public AprilTagVision(HardwareMap hwMap) {
        processor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(processor)
                .enableLiveView(true)
                .build();
    }

    /**
     * Attempts to update the Robot Pose using April Tags
     *
     * @return the robot pose [x, y, theta] in inches and radians
     */
    public RobotPose update() {
        ArrayList<AprilTagDetection> detections = processor.getDetections();
        double[] poseSum = new double[4];
        sumOfWeights = 0;
        usefulDetections = 0;

        for(AprilTagDetection detection : detections) {
            double[] currentPose = addWeights(localize(detection));

            if(currentPose != null) {
                usefulDetections++;
                poseSum[0] += currentPose[0];
                poseSum[1] += currentPose[1];
                poseSum[2] += currentPose[2];
                poseSum[3] += currentPose[3];
            }
        }

        if(usefulDetections == 0)
            return null;

        return new RobotPose(poseSum[0] / sumOfWeights, poseSum[1] / sumOfWeights,
                Math.atan2(poseSum[3], poseSum[2]));
    }

    private double[] localize(AprilTagDetection aprilTagDetection) {
        if(aprilTagDetection.metadata == null)
            return null;

        int id = aprilTagDetection.id;
        boolean isScoringTag = id <= 6;

        double range = aprilTagDetection.ftcPose.range;
        double yaw = aprilTagDetection.ftcPose.yaw;
        double bearing = aprilTagDetection.ftcPose.bearing;

        double cameraDeltaX = range * Math.cos(bearing - yaw);
        double cameraDeltaY = range * Math.sin(bearing - yaw);

        double cameraX = isScoringTag ? cameraDeltaX - 60.25 : 70.25 - cameraDeltaX;
        double cameraY = isScoringTag ? getTagY(id) + cameraDeltaY : getTagY(id) - cameraDeltaY;
        double fieldHeading = isScoringTag ? -yaw - Math.PI : -yaw;

        double localizedX = cameraX - FORWARD_OFFSET * Math.cos(fieldHeading);

        double localizedY = cameraY - FORWARD_OFFSET * Math.sin(fieldHeading);

        return new double[]{localizedX, localizedY, fieldHeading, range, id, yaw};
    }

    private double getTagY(int id) {
        switch(id) {
            case 1:
                return -41.41;
            case 2:
                return -35.41;
            case 3:
                return -29.41;
            case 4:
                return 29.41;
            case 5:
                return 35.41;
            case 6:
                return 41.41;
            case 7:
                return 40.625;
            case 8:
                return 35.125;
            case 9:
                return -35.125;
            default:
                return -40.625;
        }
    }

    private double[] addWeights(double[] data) {
        if(data == null)
            return null;

        double weighting = Math.cos(data[5]) *
                (data[4] == 7.0 || data[4] == 10.0 ? BIG_TAG_AREA : SMALL_TAG_AREA) /
                (data[3] * data[3]);

        sumOfWeights += weighting;

        return new double[]{
                weighting * data[0],
                weighting * data[1],
                weighting * Math.cos(data[2]),
                weighting * Math.sin(data[2])
        };
    }

    /**
     * Returns the number of useful April Tag Detections
     *
     * @return the number of useful detections
     */
    public int getNumDetections() {
        return usefulDetections;
    }
}