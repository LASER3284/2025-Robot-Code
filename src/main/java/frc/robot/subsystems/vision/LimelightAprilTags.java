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

    private HttpCamera llFeed;
    private MjpegServer server;
    
    private LimelightHelpers.LimelightResults llResults;
    private boolean hasPosition = false;
    private Pose2d position = empty_Pose2d;

    private NetworkTable table;
    private NetworkTableEntry tv;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry pipeline;
    
    private ScoringMode currentScoringMode = ScoringMode.Undefined;

    /* Creates a new Limelight object for AprilTag alignment. */
    public LimelightAprilTags() {
        configureNetworkTableEntries();
        configureShuffleboard();

        LimelightHelpers.getLatestResults(LimelightName);
    }

    private void configureNetworkTableEntries() {
        table = NetworkTableInstance.getDefault().getTable(LimelightName);
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        pipeline = table.getEntry("pipeline");
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab;
        tab = Shuffleboard.getTab(LimelightName);
        
        llFeed = new HttpCamera(LimelightName, "http://10.32.84.11:5800/stream.mjpg");
        server = CameraServer.addSwitchedCamera("Object Camera");
        server.setSource(llFeed);
        llFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        tab.add(server.getSource())
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(5, 0)
            .withSize(5, 5)
            .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

        tab.addString("Robot Pose", () -> getPose2d().toString());

    }

    /* Reads and stores values periodically. */
    public void periodic() {
        if (currentScoringMode == ScoringMode.AprilTag) {
            llResults = LimelightHelpers.getLatestResults(LimelightName);
        }
    }

    public Pose2d getPose2d() {
        return position;
    }

    public void setScoringMode(ScoringMode mode) {
        currentScoringMode = mode;

        if (currentScoringMode == ScoringMode.AprilTag) {
            pipeline.setDouble(1);
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