package frc.robot.subsystems.vision;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.vision.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.LimelightHelpers.LimelightTarget_Detector;

public class LimelightObjectDetection extends SubsystemBase {
    
    private final String LimelightName = "limelight_objectd";
    private final String AlgaeLabel = "algae";
    private final String CoralLabel = "coral";

    private HttpCamera llFeed;
    private MjpegServer server;

    private LimelightHelpers.LimelightResults llResults;
    private boolean detectionOn = false;

    private boolean algaeTarget = false;
    private boolean coralTarget = false;
    private Translation2d algaePose2d = null;
    private Translation2d coralPose2d = null;
    private double algaeConfidence = 0.0;
    private double coralConfidence = 0.0;

    /* Initializes a new Limelight object for gamepiece detection. */
    public LimelightObjectDetection() {
        configureShuffleboard();
        // LimelightHelpers.getFirstParse();
    }

    /* Configures the Shuffleboard to receive Limelight feed. */ 
    private void configureShuffleboard() {
        ShuffleboardTab tab;
        tab = Shuffleboard.getTab(LimelightName);

        llFeed = new HttpCamera(LimelightName, "http://10.32.84.12:5800/stream.mjpg");
        server = CameraServer.addSwitchedCamera("Object Camera");
        server.setSource(llFeed);
        llFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        tab.add(server.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(5,0)
            .withSize(5, 5)
            .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

        tab.addBoolean("Algae Detected", () -> algaeTarget);
        tab.addBoolean("Coral Detected", () -> coralTarget);

        tab.addDouble("Algae X", () -> algaePose2d != null ? algaePose2d.getX() : 0);
        tab.addDouble("Algae Y", () -> algaePose2d != null ? algaePose2d.getY() : 0);
        tab.addDouble("Coral X", () -> coralPose2d != null ? coralPose2d.getX() : 0);
        tab.addDouble("Coral Y", () -> coralPose2d != null ? coralPose2d.getY() : 0);

        tab.addDouble("Algae Confidence", () -> algaeConfidence);
        tab.addDouble("Coral Confidence", () -> coralConfidence);
        // More may be added after this for Competition Shuffleboard, should it be deemed necessary.
    }

    /* Reads and stores values periodically. */
    public void periodic() {
        if (detectionOn) {
            // Note: Because parsing the JSON takes 2.5 milliseconds, only do it when needed. 
            llResults = LimelightHelpers.getLatestResults(LimelightName);
            processLlResults(llResults);

            if (llResults != null) {
                System.out.println(LimelightHelpers.getJSONDump(LimelightName));
            }
        } else {
            algaeTarget = false;
            coralTarget = false;
            algaePose2d = null;
            coralPose2d = null;
            algaeConfidence = 0.0;
            coralConfidence = 0.0;
        }
    }

    /* Turns object detection on. */
    public void doDetection() {
        detectionOn = true;
    }

    /* Turns object detection off. */
    public void stopDetection() {
        detectionOn = false;
    }

    /* Is the camera detecting an Algae gamepiece? */
    public boolean hasAlgaeTarget() {
        return algaeTarget;
    }

    /* Locates and pinpoints the nearest Algae gamepiece. */
    public Translation2d getNearestAlgaeTarget() {
        return algaePose2d;
    }

    /* Is the camera detecting a Coral gamepiece? */
    public boolean hasCoralTarget() {
        return coralTarget;
    }

    /* Locates and pinpoints the nearest Coral gamepiece. */ 
    public Translation2d getNearestCoralTarget() {
        return coralPose2d;
    }

    public LimelightTarget_Detector[] fetchTargetsDetector() {
        return llResults.targets_Detector;
    }
    
    /* Processes the Limelight target results into Shuffleboard. */
    private void processLlResults(LimelightResults llResults) {
        LimelightTarget_Detector[] detections = fetchTargetsDetector();
        // Change variables to necessary values.
        double algaeHeight = 0;
        double coralHeight = 0;

        algaeTarget = coralTarget = false;

        if (detections != null) {
            System.out.println(" Targets detected - " + detections.length);
            for (LimelightTarget_Detector detection : detections) {
                System.out.println(" Target - " + detection.className + " Area: " + detection.ta);
                if(AlgaeLabel.equals(detection.className)) {
                    if (detection.ty < algaeHeight) {
                        algaePose2d = new Translation2d(detection.tx, detection.ty);
                        algaeHeight = detection.ty;
                        algaeConfidence = detection.confidence;
                        algaeTarget = true;
                    } 
                } else if (CoralLabel.equals(detection.className)) {
                    if (detection.ty < coralHeight) {
                        coralPose2d = new Translation2d(detection.tx, detection.ty);
                        coralHeight = detection.ty;
                        coralConfidence = detection.confidence;
                        coralTarget = true;
                    }
                }
            }
        } 
    }
}