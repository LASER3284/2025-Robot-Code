// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.*;
import frc.robot.commands.AutoAlign2;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.PreScore;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.algae_intake.AlgaeDeploy;
import frc.robot.commands.coral_intake.PivotDeploy;
import frc.robot.commands.elevator.L2;
import frc.robot.commands.elevator.ScoreOnReef;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.ToHome;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;
import frc.robot.subsystems.vision.LimelightHelpers;


public class RobotContainer { 
    public final Rollers rollers = Rollers.getInstance();
    public final IntakeRollers irollers = new IntakeRollers();
    public final Climb climb = new Climb();
    public final LimelightHelpers ll_helpers = new LimelightHelpers();
    public final Carriage carriage = new Carriage();

    public final JS js = JS.getInstance();
    public final Pivot pivotIntake = Pivot.getInstance();
    //public final JS js = new JS(rollers);
    
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
    private final CommandXboxController operator = new CommandXboxController(1);


    public final Drivetrain drivetrain = SwerveConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final AlgaeIntake algaeintake = new AlgaeIntake();
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

        // driver.b().onTrue(
        //     new AlgaeDeploy(algaeintake, Inches.of(-31))
        //     .andThen(algaeintake.rollerSpeed_Command(-.5))
        //     .andThen(new PivotToAngle(js, rollers, 0.4, 0, 0.3)).until(() -> js.isAtSetpoint(0.4)));
        // driver.b().onFalse(
        //     new PivotToAngle(js, rollers, 0.6, 0, 0)
        //     .andThen(new AlgaeDeploy(algaeintake, Inches.of(0)))
        //     .andThen(carriage.carriageCommand(5))
        //     .andThen(elevator.elevatorCommand(5)));

       //driver.start().onTrue(algaeintake.zero_command());

       //driver.a().onTrue(new CoralIntake(js, rollers, irollers, pivotIntake, elevator, 0.0, 0.0));

    //keaton comment out for now    driver.rightBumper().onTrue(new AutoAlign2(drivetrain, ll_helpers, -12.5));
       // driver.leftBumper().onTrue(new AutoAlign2(drivetrain, ll_helpers, 12.5));

       

        //operator.b().onTrue(new PreScore());
        operator.povLeft().onTrue(new ScoreOnReef(0.37, 6, .1)); //l1
        operator.povDown().onTrue(new ScoreOnReef(0.37, 13.5, .1)); //l2 and prescore
        operator.povRight().onTrue(new ScoreOnReef(0.37 , 15, 9)); //l3
        operator.povUp().onTrue(new ScoreOnReef(0.37, 20.5, 19.5)); //l4
        
        operator.x().onTrue(new ToHome()); //home
        //driver.y().onTrue(new ScoreOnReef(15, 10));

        // driver.a().onTrue(new PivotDeploy(pivotIntake, 0.3).until(() -> pivotIntake.isAtSetpoint(0.3))
        // .andThen(irollers.setMotorSpeed_command(0.3)));

        driver.a().onTrue(new CoralIntake(elevator, 0.8, 0.3));
        driver.b().onTrue(irollers.setMotorSpeed_command(0.5).andThen(rollers.coral_roller_on_command(0.5)));

        operator.rightBumper().whileTrue(rollers.coral_roller_on_command(-0.6));
        operator.rightBumper().whileFalse(rollers.coral_roller_on_command(0.05));

        driver.rightTrigger().onTrue(new SourceIntake());
    //    rollers.setDefaultCommand(rollers.coral_roller_on_command(.02));
        //driver.rightBumper().onTrue(new AutoAlign(drivetrain, ll_helpers, 12, 0));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }   
}
