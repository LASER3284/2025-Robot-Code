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

    private HttpCamera m_llFeed;
    private MjpegServer m_server;

    private LimelightHelpers.LimelightResults m_llResults;
    private boolean m_detectionOn = false;

    private boolean m_algaeTarget = false;
    private boolean m_coralTarget = false;
    private Translation2d m_algaePose2d = null;
    private Translation2d m_coralPose2d = null;
    private double m_algaeConfidence = 0.0;
    private double m_coralConfidence = 0.0;

    // Initializes a new Limelight object for gamepiece detection.
    // "LimelightHelpers.getFirstParse()" may be deleted.
    public LimelightObjectDetection() {
        configureShuffleboard();
        // LimelightHelpers.getFirstParse();
    }

    // Configures the Shuffleboard to receive Limelight streams. 
    private void configureShuffleboard() {
        ShuffleboardTab tab;
        tab = Shuffleboard.getTab(LimelightName);

        m_llFeed = new HttpCamera(LimelightName, "http://10.17.32.12:5800/stream.mjpg");
        m_server = CameraServer.addSwitchedCamera("Object Camera");
        m_server.setSource(m_llFeed);
        m_llFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        tab.add(m_server.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(5,0)
            .withSize(5, 5)
            .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

        tab.addBoolean("Algae Detected", () -> m_algaeTarget);
        tab.addBoolean("Coral Detected", () -> m_coralTarget);

        tab.addDouble("Algae X", () -> m_algaePose2d != null ? m_algaePose2d.getX() : 0);
        tab.addDouble("Algae Y", () -> m_algaePose2d != null ? m_algaePose2d.getY() : 0);
        tab.addDouble("Coral X", () -> m_coralPose2d != null ? m_coralPose2d.getX() : 0);
        tab.addDouble("Coral Y", () -> m_coralPose2d != null ? m_coralPose2d.getY() : 0);

        tab.addDouble("Algae Confidence", () -> m_algaeConfidence);
        tab.addDouble("Coral Confidence", () -> m_coralConfidence);
        // More may be added after this for Competition Shuffleboard, should it be deemed necessary.
    }

    // Reads and stores values periodically.
    public void periodic() {
        if (m_detectionOn) {
            // Note: Because parsing the JSON takes 2.5 milliseconds, only do it when needed. 
            m_llResults = LimelightHelpers.getLatestResults(LimelightName);
            processLlResults(m_llResults);

            if (m_llResults != null) {
                System.out.println(LimelightHelpers.getJSONDump(LimelightName));
            }
        } else {
            m_algaeTarget = false;
            m_coralTarget = false;
            m_algaePose2d = null;
            m_coralPose2d = null;
            m_algaeConfidence = 0.0;
            m_coralConfidence = 0.0;
        }
    }

    // Turns object detection on.
    public void doDetection() {
        m_detectionOn = true;
    }

    // Turns object detection off.
    public void stopDetection() {
        m_detectionOn = false;
    }

    // Is the camera detecting an Algae gamepiece?
    public boolean hasAlgaeTarget() {
        return m_algaeTarget;
    }

    // Locates and pinpoints the nearest Algae gamepiece.
    public Translation2d getNearestAlgaeTarget() {
        return m_algaePose2d;
    }

    // Is the camera detecting a Coral gamepiece?
    public boolean hasCoralTarget() {
        return m_coralTarget;
    }

    // Locates and pinpoints the nearest Coral gamepiece.
    public Translation2d getNearestCoralTarget() {
        return m_coralPose2d;
    }
    
    // Redefine or remove this method.
    // private LimelightTarget_Detector[] fetchTargetDetector() {

    // Processes the Limelight target results into Shuffleboard.
    private void processLlResults(LimelightResults m_llResults) {
        // Redefine or remove this variable (see line 121.)
        LimelightTarget_Detector[] detections = fetchTargetDetector();
        // Change variables to necessary values.
        double m_algaeHeight = 0;
        double m_coralHeight = 0;

        m_algaeTarget = m_coralTarget = false;

        // Everything below this line may be modified or deleted.
        if (detections != null) {
            System.out.println(" Targets detected - " + detections.length);
            for (LimelightTarget_Detector detection : detections) {
                System.out.println(" Target - " + detection.className + " Area: " + detection.ta);
                if(AlgaeLabel.equals(detection.className)) {
                    if (detection.ty < m_algaeHeight) {
                        m_algaePose2d = new Translation2d(detection.tx, detection.ty);
                        m_algaeHeight = detection.ty;
                        m_algaeConfidence = detection.confidence;
                        m_algaeTarget = true;
                    } 
                } else if (CoralLabel.equals(detection.className)) {
                    if (detection.ty < m_coralHeight) {
                        m_coralPose2d = new Translation2d(detection.tx, detection.ty);
                        m_coralHeight = detection.ty;
                        m_coralConfidence = detection.confidence;
                        m_coralTarget = true;
                    }
                }
            }
        } 
    }
}