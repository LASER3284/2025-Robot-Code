// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.SwerveConstants;
//import frc.robot.commands.algae_intake.AlgaeDeploy;
//import frc.robot.commands.algae_intake.AlgaeIntakeCommand;
//import frc.robot.commands.algae_intake.AlgaeStow;
//import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climb;

public class RobotContainer {
    private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //private final AlgaeIntake algaeintake = new AlgaeIntake();

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Climb climb = new Climb();
   


   // public final Drivetrain drivetrain = SwerveConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
       // drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
         //   drivetrain.applyRequest(() ->
              //  drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                  //  .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    //.withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
       //     )
      //  );

    //    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
       // joystick.b().whileTrue(drivetrain.applyRequest(() ->
            //point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
     //   ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
     //   joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      //  joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
       // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
       // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

       // drivetrain.registerTelemetry(logger::telemeterize);

       //joystick.a().onTrue(algaeintake.stopmotor());
       //joystick.b().onTrue(new AlgaeDeploy(algaeintake, Inches.of(7)));
      // joystick.x().onTrue(new AlgaeDeploy(algaeintake, Inches.of(4)));
      // joystick.y().onTrue(new AlgaeStow(algaeintake, Inches.of(-4)));

       // joystick.b().whileTrue(new AlgaeDeploy(algaeintake, Inches.of(7))
           // .andThen(algaeintake.rollerSpeed_Command(.5)));
       // joystick.b().whileFalse(new AlgaeStow(algaeintake, Inches.of(-4)));

        
    

       //joystick.povUp().onTrue(algaeintake.zero_command());
    //    joystick.a().whileTrue(Climb(climb, Degrees.of(180)));
    //           climb.rollerSpeed_Command(.5);
    //           joystick.b().whileFalse(Climb(climb,Degrees.of(0)));
    //                  climb.rollerSpeed_Command(0);
    //               }
    //               private Command Climb(Climb climb, Angle degrees) {
    //                 throw new UnsupportedOperationException("Unimplemented method 'Climb'");

    //        }
       

        joystick.a().whileTrue(climb.climbspeedCommand(0.5));
        joystick.a().whileFalse(climb.climbspeedCommand(0));
        joystick.b().whileTrue(climb.stopmotor(0));
        joystick.b().whileFalse(climb.stopmotor(0));
    }
                public Command getAutonomousCommand() {
            return Commands.print("No autonomous command configured");
    }
   
}
