// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.*;
import frc.robot.commands.AutoAlign2;
//import frc.robot.commands.CoralIntake;
//import frc.robot.commands.SourceIntake;
import frc.robot.commands.algae_intake.AlgaeDeploy;
import frc.robot.commands.algae_intake.AlgaeStow;
import frc.robot.commands.coral_intake.PivotDeploy;
import frc.robot.commands.pivot.PivotToAngle;
//import frc.robot.commands.coral_intake.PivotDeploy;
//import frc.robot.commands.pivot.PivotToAngle;
//import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
//import frc.robot.subsystems.Rollers;
//import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;
//import frc.robot.subsystems.pivotintake.Pivot;
import frc.robot.subsystems.vision.LimelightHelpers;

public class RobotContainer { 
    public final JS js = new JS();
    //public final Pivot pivotIntake = new Pivot();

    //public final Rollers rollers = new Rollers();
    public final IntakeRollers irollers = new IntakeRollers();
    public final Climb climb = new Climb();
    public final LimelightHelpers ll_helpers = new LimelightHelpers();
    
    SendableChooser<Command> autoChooser;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private final CommandXboxController driver = new CommandXboxController(0);
    private final EventLoop loop = new EventLoop();

    private final BooleanEvent elevator_event = new BooleanEvent(loop, driver.povDown());

    public final Drivetrain drivetrain = SwerveConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Carriage carriage = new Carriage();
    public final AlgaeIntake algaeintake = new AlgaeIntake();
    public final Rollers rollers = new Rollers();

    public LimelightHelpers cams = new LimelightHelpers();
    
    public final Climb climber = new Climb();
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
                drive.withVelocityX(-driver.getLeftY() * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond)) 
                    .withVelocityY(-driver.getLeftX() * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond)) 
                    .withRotationalRate(-driver.getRightX() * RotationsPerSecond.of(0.75).in(RadiansPerSecond))) 
            );
    

    //   driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //    driver.b().whileTrue(drivetrain.applyRequest(() ->
    //         point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
    //    ));

        driver.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                .withVelocityY(-driver.getLeftX() * SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                .withRotationalRate(-driver.getRightX() * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.5))
                );


       // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
       // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
       // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //    driver.b().whileTrue(new AlgaeDeploy(algaeintake, Inches.of(-31))
    //        .andThen(algaeintake.rollerSpeed_Command(-.5)));
    //    driver.b().onFalse(new AlgaeDeploy(algaeintake, Inches.of(0)));

        driver.start().onTrue(algaeintake.zero_command());

        // driver.a().whileTrue(carriage.carriageCommand(4).andThen(elevator.elevatorCommand(4)));
         //driver.b().whileTrue(carriage.carriageCommand(5));
         //driver.x().whileTrue(carriage.carriageCommand(3).andThen(elevator.elevatorCommand(3)));
        // POSITIVE TOWARDS THE BACK
        //driver.a().whileTrue(js.setSpeed_command(js.calculateJSPose(Degrees.of(0.3))));
        // NEGATIVE TOWARDS THE FRONT
       // driver.x().whileTrue(js.setSpeed_command(js.calculateJSPose(Degrees.of(0.3))));
        // driver.b().whileTrue(new PivotDeploy(pivotIntake, Degrees.of(-8))
        // .andThen(js.setSpeed_command(js.calculateJSPose(Degrees.of(0.3)))
        // .andThen(new WaitCommand(0.5))
        // .andThen(carriage.carriageCommand(3))
        // .andThen(rollers.coral_roller_on_command(0.5))
        // .andThen(irollers.setMotorSpeed_command(5))));
        // driver.b().whileFalse(carriage.carriageCommand(-2)
        // .andThen(rollers.coral_roller_on_command(0))
        // .andThen(irollers.setMotorSpeed_command(0))
        // .andThen(js.setSpeed_command(js.calculateJSPose(Degrees.of(0.3))))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new PivotDeploy(pivotIntake, Degrees.of(-0.1))));
        // driver.y().onTrue(irollers.setMotorSpeed_command(50));
        // driver.y().onFalse(irollers.setMotorSpeed_command(0));
        
        driver.b().whileTrue(new PivotToAngle(js, rollers, Degrees.of(0.75), 0));
        driver.x().whileTrue(new PivotToAngle(js, rollers, Degrees.of(0.55), 0)); //.until(() -> js.isAtSetpoint(0.55));

        // driver.b().whileTrue(js.calcCommand(0.55));
        // driver.x().whileTrue(js.calcCommand(0.75));

        driver.y().whileTrue(rollers.coral_roller_on_command(0.5));
        driver.y().whileFalse(rollers.coral_roller_on_command(0));


        driver.rightBumper().onTrue(new AutoAlign2(drivetrain, ll_helpers, -12.5));
        driver.leftBumper().onTrue(new AutoAlign2(drivetrain, ll_helpers, 12.5));
        //driver.rightBumper().onTrue(new AutoAlign(drivetrain, ll_helpers, 12, 0));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }   
}
