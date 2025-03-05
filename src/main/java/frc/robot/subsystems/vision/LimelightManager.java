package frc.robot.subsystems.vision;

public class LimelightManager {

    public static double getTX(String limelightName) {
        return LimelightHelpers.getTX(limelightName);
    }

    public static double getTY(String limelightName) {
        return LimelightHelpers.getTY(limelightName);
    }

    public static boolean hasTarget(String limelightName) {
        return LimelightHelpers.getTV(limelightName);
    }
}
