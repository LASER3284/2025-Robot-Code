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
import frc.robot.commands.AlgaePreScore;
import frc.robot.commands.AlgaeReefHigh;
import frc.robot.commands.AlgaeReefLow;
import frc.robot.commands.AutoAlign3;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.PreScore;
import frc.robot.commands.ProcessorPreScore;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.ScoreOnL1;
import frc.robot.commands.ScoreOnReef;
import frc.robot.commands.ScoreOnReefAuto;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.SourceIntakeRetract;
import frc.robot.commands.NetScore;
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
import frc.robot.subsystems.vision.LimelightAlignment;


public class RobotContainer { 
    public static double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 


    // subsystem instantiation
    public final Drivetrain drivetrain = SwerveConstants.createDrivetrain();
    public final Rollers rollers = Rollers.getInstance();
    public final IntakeRollers irollers = IntakeRollers.getInstance();
    public final Carriage carriage = Carriage.getInstance();
    public final JS js = JS.getInstance();
    public final Pivot pivotIntake = Pivot.getInstance();
    public final Elevator elevator = Elevator.getInstance();
    public final AlgaeIntake algaeintake = AlgaeIntake.getInstance();

    public final LimelightAlignment alignment = new LimelightAlignment();
    

    
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

    public RobotContainer() {
    
        // register commands to pathplanner
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

        // NamedCommands.registerCommand("right align", alignment.LimelightAlign(drivetrain, true, "limelight-right").withTimeout(2));
        // NamedCommands.registerCommand("left align", alignment.LimelightAlign(drivetrain, false, "limelight-left").withTimeout(2));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        drivetrain.addVisionMeasurement(drivetrain.getState().Pose, 0);

        configureBindings();
        
    }
    
    private void configureBindings() {

        drivetrain.registerTelemetry(logger::telemeterize);

        // DRIVER CONTROLS \\

        // driving
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) 
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) 
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)) 
            );

        // slow button
        driver.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.15)
                .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.15)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.15))
                );

        // tb tested... 5986 robot centric stuff        
        driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        driver.pov(90).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0).withVelocityY(0.5)));
        driver.pov(270).whileTrue(drivetrain.applyRequest(() ->
                forwardStraight.withVelocityX(0).withVelocityY(-.5)));

        // coral ground intake to score l1 pose
        // driver.rightBumper().whileTrue(new CoralIntake());
        // driver.rightBumper().whileFalse(new ScoreOnL1());

        // coral ground intake rollers on/off
        driver.b().onTrue(irollers.setMotorSpeed_command(.8));
        driver.b().onFalse(irollers.setMotorSpeed_command(0));

        // algae ground intake to prescore processor/prep for barge score
        // driver.leftBumper().onTrue(new AlgaePreScore());
        // driver.leftBumper().onFalse(new ProcessorPreScore());
        // driver.leftBumper().onTrue(alignment.setYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
        // driver.leftBumper().whileTrue(alignment.LimelightAlign(drivetrain, true, "limelight-left"));


        // driver.rightBumper().onTrue(alignment.setYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
        // driver.rightBumper().whileTrue(alignment.LimelightAlign(drivetrain, false, "limelight-right"));



        // OPERATOR CONTROLS \\

        // score l1
        operator.povLeft().onTrue(new ScoreOnL1());

        // score l2
        operator.povDown().onTrue(new ScoreOnReef(0.34, 13, -0.3)); 

        // score l3
        operator.povRight().onTrue(new ScoreOnReef(0.34 , 14.25, -8));

        // score l4
        operator.povUp().onTrue(new ScoreOnReef(0.34, 18.5, -19 ));

        // algae score in net
        operator.y().onTrue(new NetScore());

        // grab high algae off reef
        operator.povUp().and(operator.rightTrigger(.5)).onTrue(new AlgaeReefHigh());

        // grab low algae off reef
        operator.povRight().and(operator.rightTrigger(.5)).onTrue(new AlgaeReefLow());

        // stow
        operator.x().onTrue(new ToHome());

        // score in processor
        operator.leftBumper().whileTrue(new ProcessorScore());
        operator.leftBumper().onFalse(rollers.algae_roller_on_command(0));


        // spit out coral, both from js and from ground intake
        operator.rightBumper().whileTrue(rollers.coral_roller_on_command(-0.6).andThen(irollers.setMotorSpeed_command(-0.5)));
        operator.rightBumper().whileFalse(rollers.coral_roller_on_command(0.05).andThen(irollers.setMotorSpeed_command(0)));

        // source intake and then stow back
        operator.leftTrigger().onTrue(new SourceIntake(0.705, 0.5));
        operator.leftTrigger().onFalse(new SourceIntakeRetract(0.6, 0));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }   
}
