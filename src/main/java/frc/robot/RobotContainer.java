// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.*;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.algae_intake.AlgaeDeploy;
import frc.robot.commands.coral_intake.PivotDeploy;
import frc.robot.commands.pivot.PivotToAngle;

import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;
import frc.robot.subsystems.vision.LimelightHelpers;

public class RobotContainer { 

    public final JS js = new JS();
    public final Rollers rollers = new Rollers();
    public final IntakeRollers irollers = new IntakeRollers();
    public final Climb climb = new Climb();

    public final Pivot p_intake = new Pivot();
    
    SendableChooser<Command> autoChooser;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Elevator.MaxSpeed * 0.1).withRotationalDeadband(Elevator.MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(Elevator.MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    public final Drivetrain drivetrain = SwerveConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final AlgaeIntake algaeintake = new AlgaeIntake();

    public LimelightHelpers cams = new LimelightHelpers();
    
    //public final Climb climber = new Climb();
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        drivetrain.addVisionMeasurement(drivetrain.getState().Pose, 0);

        configureBindings();
        
    }
    
    private void configureBindings() {

        drivetrain.registerTelemetry(logger::telemeterize);

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * Elevator.MaxSpeed) 
                    .withVelocityY(-driver.getLeftX() * Elevator.MaxSpeed) 
                    .withRotationalRate(-driver.getRightX() * Elevator.MaxAngularRate)) 
            );
    

    //   driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //    driver.b().whileTrue(drivetrain.applyRequest(() ->
    //         point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
    //    ));

        driver.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * Elevator.MaxSpeed * 0.1)
                .withVelocityY(-driver.getLeftX() * Elevator.MaxSpeed * 0.1)
                .withRotationalRate(-driver.getRightX() * Elevator.MaxAngularRate * 0.5))
                );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        // driver.pov(0).whileTrue(drivetrain.applyRequest(() -> 
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0)));

        // driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-.05).withVelocityY(0)));


       // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
       // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
       // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.back().onTrue(rollers.coral_roller_on_command(-.5));
        driver.back().onFalse(rollers.coral_roller_on_command(0));

        //driver.start().onTrue(rollers.algae_roller_on_command(1));
        //driver.start().onFalse(rollers.algae_roller_on_command(0));

        driver.start().onTrue(rollers.coral_roller_on_command(.5));
        driver.start().onFalse(rollers.coral_roller_on_command(0));

    //    driver.b().whileTrue(new AlgaeDeploy(algaeintake, Inches.of(31))
    //        .andThen(algaeintake.rollerSpeed_Command(-.5)));
    //    driver.b().whileFalse(new AlgaeStow(algaeintake, Inches.of(0)));

        // driver.y().onTrue(pivot.setSpeed_command(-0.1));
        // driver.y().onFalse(pivot.setSpeed_command(0));
        driver.y().onTrue(p_intake.zero_Command()
            .andThen(js.zero_command())
            .andThen(elevator.zero_command()));

        driver.x().onTrue(new PivotDeploy(p_intake, Degrees.of(11))
            .andThen(irollers.setMotorSpeed_command(0.4)));
        driver.x().onFalse(new PivotDeploy(p_intake, Degrees.of(0))
            .andThen(irollers.setMotorSpeed_command(0))
            .andThen(p_intake.zero_Command()));

        driver.b().onTrue(rollers.coral_roller_on_command(-0.4)
            .andThen(new PivotToAngle(js, rollers, Degrees.of(0.06), 0.4)));

        //driver.povLeft().onTrue(new PivotToAngle(js, Degrees.of(-0.15)));

        driver.povRight().onTrue(new CoralIntake(js, rollers, irollers, p_intake, -0.25, -7.5)
            .andThen(irollers.setMotorSpeed_command(0.4)
            .andThen(rollers.coral_roller_on_command(-0.8)))
            .andThen(new PivotToAngle(js, rollers, Degrees.of(0.2), 0)));
        driver.povLeft().onTrue(irollers.setMotorSpeed_command(0)
            .andThen(rollers.coral_roller_on_command(0))
            .andThen(new PivotToAngle(js, rollers, Degrees.of(-0.2), 0)
            .andThen(new PivotDeploy(p_intake, Degrees.of(0)))));

        // driver.povUp().onTrue(new PivotToAngle(js, rollers, Degrees.of(-0.25))
        //     .andThen(new AlgaeDeploy(algaeintake, Inches.of(31)))
        //     .andThen(algaeintake.rollerSpeed_Command(-.5))
        //     .andThen(rollers.algae_roller_on_command(0.5)));
        // driver.povDown().onTrue(new AlgaeDeploy(algaeintake, Inches.of(0))
        //     .andThen(new PivotToAngle(js, rollers, Degrees.of(0))));

        driver.povUp().onTrue(climb.climbspeedCommand(0.5));
        driver.povUp().onFalse(climb.climbspeedCommand(0));

        driver.povDown().onTrue(climb.climbspeedCommand(-0.5));
        driver.povDown().onFalse(climb.climbspeedCommand(0));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }   
}
