package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Drivetrain;

public class Constants {
    public static class SwerveConstants {
        private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.59).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;

        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;


    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;

    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    private static final Current kSlipCurrent = Amps.of(120.0);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    private static final Pigeon2Configuration pigeonConfigs = null;

    public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(3.92);

    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.12;
    private static final double kSteerGearRatio = 12.8;
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 62;

    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 3;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.054443359375);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(13.25);
    private static final Distance kFrontLeftYPos = Inches.of(13.25);

    // Front Right
    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 5;
    private static final int kFrontRightEncoderId = 6;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.343994140625);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(13.25);
    private static final Distance kFrontRightYPos = Inches.of(-13.25);

    // Back Left
    private static final int kBackLeftDriveMotorId = 7;
    private static final int kBackLeftSteerMotorId = 8;
    private static final int kBackLeftEncoderId = 9;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.449462890625);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-13.25);
    private static final Distance kBackLeftYPos = Inches.of(13.25);

    // Back Right
    private static final int kBackRightDriveMotorId = 10;
    private static final int kBackRightSteerMotorId = 11;
    private static final int kBackRightEncoderId = 12;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.158203125);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-13.25);
    private static final Distance kBackRightYPos = Inches.of(-13.25);

    // Field
    public static final Field2d field = new Field2d();


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    public static Drivetrain createDrivetrain() {
        return new Drivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }

    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}

    public static class ElevatorConstants {

        //trapezoid profile
        public static final double maxVelocity = 2;
        public static final double maxAcceleration = 1;

        //can ids
        public static final int ER_ID = 2;
        public static final int EL_ID = 3;

        // heights
        public static final Distance HANDOFF_HEIGHT = Units.Inches.of(4);
        public static final Distance L2_HEIGHT = Units.Inches.of(7);
        public static final Distance L3_HEIGHT = Units.Inches.of(12);
        public static final Distance L4_HEIGHT = Units.Inches.of(15);

        //gearing
        public static final double GEAR_RATIO = 1/ 6.67 ;
        public static final double PULLEY = 1.88;
        public static final double TOLERANCE = 0.1;
        public static final double LINEAR_DISTANCE_CONST = PULLEY * Math.PI;

        //pid
        public static final double P = 0.4;
        public static final double I = 0;
        public static final double D = 0.0;

        //ff
        public static final double kS = 0.2234;
        public static final double kG = 0.0456;
        public static final double kV = 0.4567;
        public static final double kA = 0.0128;
    }

    public static class AlgaeIntakeConstants {
        // can ids
        public static final int ARACK_ID = 13;
        public static final int AROLLER_ID = 14;

        // pid
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        // motor
        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;

        //ff
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        // di
        public static final int DI_LIMIT_PORT = 9;

        public static final double GEAR_RATIO = 0;

        public static final Distance DEPLOYED = Inches.of(0);
        public static final Distance ZERO = Inches.of(0);
    }
}