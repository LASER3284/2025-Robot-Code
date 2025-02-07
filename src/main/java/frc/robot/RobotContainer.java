// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.elevator.ToPosition;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.commands.algae_intake.AlgaeDeploy;
import frc.robot.commands.algae_intake.AlgaeIntakeCommand;
import frc.robot.commands.algae_intake.AlgaeStow;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.AlgaeIntake;
//import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Rollers;
import frc.robot.commands.pivot.PivotToAngle;

public class RobotContainer {
    private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    public final Pivot pivot = new Pivot();
    public final Rollers rollers = new Rollers();
    
    SendableChooser<Command> autoChooser;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    public final Drivetrain drivetrain = SwerveConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final AlgaeIntake algaeintake = new AlgaeIntake();
    //public final Climb climber = new Climb();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        
    }
    
    private void configureBindings() {

        drivetrain.registerTelemetry(logger::telemeterize);
        
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftX() * MaxSpeed) 
                    .withVelocityY(-driver.getLeftY() * MaxSpeed) 
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)) 
            );
    

    //   driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //    driver.b().whileTrue(drivetrain.applyRequest(() ->
    //         point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
    //    ));

        driver.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftX() * 0.5)
                .withVelocityY(-driver.getLeftY() * 0.5)
                .withRotationalRate(-driver.getRightX() * 0.75))
                );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        // driver.pov(0).whileTrue(drivetrain.applyRequest(() -> 
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0)));

        // driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-.05).withVelocityY(0)));


        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // driver.povLeft().onTrue(new ToPosition(elevator, ElevatorConstants.HANDOFF_HEIGHT));
        // driver.povDown().onTrue(new ToPosition(elevator, ElevatorConstants.L2_HEIGHT));
        // driver.povRight().onTrue(new ToPosition(elevator, ElevatorConstants.L3_HEIGHT));
        // driver.povUp().onTrue(new ToPosition(elevator, ElevatorConstants.L4_HEIGHT));
        // driver.leftBumper().onTrue(new ToPosition(elevator, Inches.of(0)));

        driver.povUp().onTrue(elevator.set_power_command(0.5));
        driver.povUp().onFalse(elevator.set_power_command(0));

        driver.povDown().onTrue(elevator.set_power_command(-0.1));
        driver.povDown().onFalse(elevator.set_power_command(0));

        // driver.povUp().onTrue(elevator.set_power_command(0.25));
        // driver.povDown().onTrue(elevator.set_power_command(-0.25));
        // driver.povUp().onFalse(elevator.set_power_command(0));
        // driver.povDown().onFalse(elevator.set_power_command(0));

        driver.back().onTrue(rollers.coral_roller_on_command(0.1));
        driver.back().onFalse(rollers.coral_roller_on_command(0));

        driver.start().onTrue(rollers.algae_roller_on_command(1));
        driver.start().onFalse(rollers.algae_roller_on_command(0));

        driver.a().onTrue(pivot.setSpeed_command(-0.1));
        driver.a().onFalse(pivot.setSpeed_command(0));

        driver.b().whileTrue(new AlgaeDeploy(algaeintake, Inches.of(31))
            .andThen(algaeintake.rollerSpeed_Command(-.5)));
        driver.b().whileFalse(new AlgaeStow(algaeintake, Inches.of(0)));

        driver.y().onTrue(pivot.setSpeed_command(0.1));
        driver.y().onFalse(pivot.setSpeed_command(0));

        driver.x().onTrue(elevator.zero_command());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }   
}
