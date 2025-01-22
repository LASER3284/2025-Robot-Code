package frc.robot.subsystems.vision;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightAprilTags extends SubsystemBase {

    private final String LimelightName = "limelight-apriltags";
    private final Pose2d empty_Pose2d = new Pose2d();

    private HttpCamera m_llFeed;
    private MjpegServer m_server;
    
    private LimelightHelpers.LimelightResults m_llResults;
    private boolean m_hasPosition = false;
    private Pose2d m_position = empty_Pose2d;

    private NetworkTable m_table;
    private NetworkTableEntry m_tv;
    private NetworkTableEntry m_tx;
    private NetworkTableEntry m_ty;
    private NetworkTableEntry m_ta;
    private NetworkTableEntry m_pipeline;
    
    private ScoringMode currentScoringMode = ScoringMode.Undefined;

    /* Creates a new Limelight object for AprilTag alignment. */
    public LimelightAprilTags() {
        configureNetworkTableEntries();
        configureShuffleboard();

        LimelightHelpers.getLatestResults(LimelightName);
    }

    private void configureNetworkTableEntries() {
        m_table = NetworkTableInstance.getDefault().getTable(LimelightName);
        m_tv = m_table.getEntry("tv");
        m_tx = m_table.getEntry("tx");
        m_ty = m_table.getEntry("ty");
        m_ta = m_table.getEntry("ta");
        m_pipeline = m_table.getEntry("pipeline");
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab;
        tab = Shuffleboard.getTab(LimelightName);
        
        m_llFeed = new HttpCamera(LimelightName, "http://10.32.84.11:5800/stream.mjpg");
        m_server = CameraServer.addSwitchedCamera("Object Camera");
        m_server.setSource(m_llFeed);
        m_llFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        tab.add(m_server.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(5, 0)
            .withSize(5, 5)
            .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

        tab.addString("Robot Pose", () -> getPose2d().toString());

    }

    /* Reads and stores values periodically. */
    public void periodic() {
        if (currentScoringMode == ScoringMode.AprilTag) {
            m_llResults = LimelightHelpers.getLatestResults(LimelightName);
        }
    }

    public Pose2d getPose2d() {
        return m_position;
    }

    public void setScoringMode(ScoringMode mode) {
        currentScoringMode = mode;

        if (currentScoringMode == ScoringMode.AprilTag) {
            m_pipeline.setDouble(1);
        }
    }

    public ScoringMode getScoringMode() {
        return currentScoringMode;
    }

    public enum ScoringMode {
        AprilTag,
        Undefined;
    }
}