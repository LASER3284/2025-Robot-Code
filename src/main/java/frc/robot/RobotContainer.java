// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.*;
import frc.robot.commands.AlgaeReefHigh;
import frc.robot.commands.AlgaeReefLow;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.PreScore;
import frc.robot.commands.ProcessorPreScore;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.SourceIntakeRetract;
import frc.robot.commands.algae_intake.AlgaePreScore;
import frc.robot.commands.NetScore;
import frc.robot.commands.elevator.ScoreOnReef;
import frc.robot.commands.elevator.ScoreOnReefAuto;
import frc.robot.commands.elevator.ScoreOnL1;
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

import frc.robot.commands.vision.AutoAlign3;


public class RobotContainer { 
    public static double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

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
    public final AlgaeIntake algaeintake = AlgaeIntake.getInstance();
    public LimelightHelpers cams = new LimelightHelpers();
    
    public final Climb climber = new Climb();

    public RobotContainer() {
    

        NamedCommands.registerCommand("stow", new ToHome());
        NamedCommands.registerCommand("L2", new ScoreOnReef(0.365, 13.5, -0.3));
        NamedCommands.registerCommand("L3", new ScoreOnReef(0.365, 15, -6));
        NamedCommands.registerCommand("L4", new ScoreOnReefAuto(0.34, 19.5, -19));
        NamedCommands.registerCommand("coral intake", new CoralIntake());
        NamedCommands.registerCommand("prescore", new PreScore());
        NamedCommands.registerCommand("CORAL_SPIT", rollers.coral_roller_on_command(-0.9));
        NamedCommands.registerCommand("STOP_SPIT", rollers.coral_roller_on_command(0));
        NamedCommands.registerCommand("source intake", new SourceIntake(0.705, 0.5));
        NamedCommands.registerCommand("coral rollers", irollers.setMotorSpeed_command(-0.6));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        drivetrain.addVisionMeasurement(drivetrain.getState().Pose, 0);

        configureBindings();
        
    }
    
    private void configureBindings() {

        drivetrain.registerTelemetry(logger::telemeterize);

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) 
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) 
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)) 
            );

        driver.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.15)
                .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.15)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.15))
                );
        
        driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        driver.pov(90).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0).withVelocityY(0.5)));
        driver.pov(270).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0).withVelocityY(-.5)));


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
       // operator.a().onTrue(elevator.elevatorCommand(-12));
       
        operator.povLeft().onTrue(new ScoreOnL1()); //l1
        operator.povDown().onTrue(new ScoreOnReef(0.34, 13, -0.3)); //l2 and prescore
        operator.povRight().onTrue(new ScoreOnReef(0.34 , 14.25, -8)); //l3
        operator.povUp().onTrue(new ScoreOnReef(0.34, 18.5, -19 )); //l4
        operator.y().onTrue(new NetScore(.6, 20, -19.4)); // algae in net boi
        operator.povUp().and(operator.rightTrigger(.5)).onTrue(new AlgaeReefHigh()); //pick high algae
        operator.povRight().and(operator.rightTrigger(.5)).onTrue(new AlgaeReefLow()); //pick low algae
        operator.x().onTrue(new ToHome()); //stow

        operator.a().whileTrue(js.setSpeed_command(0));
    

        

        driver.rightBumper().whileTrue(new CoralIntake());
        driver.rightBumper().whileFalse(new ScoreOnL1());


        
        driver.b().onTrue(irollers.setMotorSpeed_command(.8));
        driver.b().onFalse(irollers.setMotorSpeed_command(0));

        // driver.a().onTrue(new AlgaeDeploy(algaeintake, Inches.of(-20)));
        // driver.a().onFalse(new AlgaeDeploy(algaeintake, Inches.of(-9)));

         //driver.a().onTrue(new ElevatorCommand(2));

      //driver.y().onTrue(new PivotToAngleEnd(js, rollers, .5, 0.0, 0.0));

        driver.leftBumper().onTrue(new AlgaePreScore());
        driver.leftBumper().onFalse(new ProcessorPreScore());

        // driver.leftBumper().onTrue(new AutoAlign3(false, drivetrain, "limelight_right"));
        // driver.rightBumper().onTrue(new AutoAlign3(true, drivetrain, "limelight_left"));
        operator.leftBumper().whileTrue(new ProcessorScore());
        operator.leftBumper().onFalse(rollers.algae_roller_on_command(0));



        //GO RIGHT
      //  driver.rightBumper().onTrue(new AutoAlign(drivetrain, ll_helpers, 0.1, 0.1, "limelight-left"));
        //GO LEFT
       // driver.leftBumper().onTrue(new AutoAlign(drivetrain, ll_helpers, 0.1, 0.1, "limelight-right"));

        

        //driver.b().onTrue(new ScoreOnReef(0.5, 20.5, 19.5));
        //driver.b().onTrue(new ElevatorCommand(2));
        //driver.b().onTrue(irollers.setMotorSpeed_command(-0.5).andThen(rollers.coral_roller_on_command(-0.5)));
        //driver.b().onTrue(irollers.setMotorSpeed_command(0.7).andThen(rollers.coral_roller_on_command(0.5)));

        operator.rightBumper().whileTrue(rollers.coral_roller_on_command(-0.6).andThen(irollers.setMotorSpeed_command(-0.5)));
        operator.rightBumper().whileFalse(rollers.coral_roller_on_command(0.05).andThen(irollers.setMotorSpeed_command(0)));



        operator.leftTrigger().onTrue(new SourceIntake(0.705, 0.5));
        operator.leftTrigger().onFalse(new SourceIntakeRetract(0.6, 0));



    //    rollers.setDefaultCommand(rollers.coral_roller_on_command(.02));
        //driver.rightBumper().onTrue(new AutoAlign(drivetrain, ll_helpers, 12, 0));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }   
}
