// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.algae_intake.AlgaeDeploy;
import frc.robot.commands.algae_intake.AlgaeStow;
import frc.robot.subsystems.AlgaeIntake;
//import frc.robot.commands.elevator.ToPosition;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    SendableChooser<Command> autoChooser;
    Field2d field = new Field2d();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    public final Drivetrain drivetrain = SwerveConstants.createDrivetrain();
    //public final Elevator elevator = new Elevator();
    public final AlgaeIntake algaeintake = new AlgaeIntake();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        //SmartDashboard.putData("field", Constants.SwerveConstants.field);

        configureBindings();
    }

    
    
    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) 
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) 
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)) 
            );

       driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
       driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
       ));

        driver.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(1)
                .withVelocityY(1)
                .withRotationalRate(1))
                );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.


        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        /* 
        // driver.povLeft().onTrue(new ToPosition(elevator, ElevatorConstants.HANDOFF_HEIGHT));
        // driver.povDown().onTrue(new ToPosition(elevator, ElevatorConstants.L2_HEIGHT));
        // driver.povRight().onTrue(new ToPosition(elevator, ElevatorConstants.L3_HEIGHT));
        // driver.povUp().onTrue(new ToPosition(elevator, ElevatorConstants.L4_HEIGHT));

        // driver.a().onTrue(algaeintake.stopmotor());
        // driver.b().onTrue(new AlgaeDeploy(algaeintake, Inches.of(7)));
        // driver.x().onTrue(new AlgaeDeploy(algaeintake, Inches.of(1)));
        // driver.y().onTrue(new AlgaeStow(algaeintake, Inches.of(-4)));
        // driver.start().onTrue(algaeintake.zero_command());
*/
        // driver.b().whileTrue(new AlgaeDeploy(algaeintake, Inches.of(21))
        //     .andThen(algaeintake.rollerSpeed_Command(.5)));
        // driver.b().whileFalse(new AlgaeStow(algaeintake, Inches.of(-4)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
   
}
