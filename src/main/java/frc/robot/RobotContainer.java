// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double SlowSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double SlowAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverCtrl = new CommandXboxController(0);
    private final CommandXboxController operatorCtrl = new CommandXboxController(1);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Intake intake = new Intake(Constants.IntakeMotorID);
    //private final Indexer indexer = new Indexer(Constants.IndexMotorID);
    //private final Shooter shooter = new Shooter(Constants.shooterMotorLID, Constants.shooterMotorRID);
    private final Cameras cameras = new Cameras(drivetrain);

    //Pathplanner
    SendableChooser<Command> autoChooser;

    // //Command Groups
    // private final Command shootCmd = Commands.parallel(
    //     shooter.setVelocityCmd(Constants.ShooterShootingVelocity),
    //     indexer.setVelocityCmd(Constants.IndexerShootingVelocity)
    // );

    // private final Command chargeCmd = Commands.parallel(
    //     shooter.setVelocityCmd(Constants.ShooterChargeVelocity),
    //     indexer.setVelocityCmd(Constants.IndexerChargeVelocity)
    // );

    public RobotContainer() {
        //intake = new Intake(5);

        NamedCommands.registerCommand("Intake Command", intake.getIntakeCmd(1));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverCtrl.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverCtrl.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverCtrl.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverCtrl.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverCtrl.getLeftY() * SlowSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverCtrl.getLeftX() * SlowSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverCtrl.getRightX() * SlowAngularRate)) // Drive counterclockwise with negative X (left)
                    
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverCtrl.back().and(driverCtrl.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverCtrl.back().and(driverCtrl.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverCtrl.start().and(driverCtrl.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverCtrl.start().and(driverCtrl.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        Command sysIdCommandGroup =  Commands.sequence(
            drivetrain.sysIdDynamic(Direction.kForward),
            drivetrain.sysIdDynamic(Direction.kReverse),
            drivetrain.sysIdQuasistatic(Direction.kForward),
            drivetrain.sysIdQuasistatic(Direction.kReverse)
        );

        //driverCtrl.start().onTrue(sysIdCommandGroup);

        // Reset the field-centric heading on left bumper press.
        driverCtrl.pov(0).onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(Meters.of(12.51), Meters.of(4.041), Rotation2d.fromDegrees(0)))));

        driverCtrl.a().whileTrue(intake.getIntakeCmd(-Constants.IntakeVelocity));
        driverCtrl.b().whileTrue(intake.getIntakeCmd(Constants.IntakeVelocity));
        // driverCtrl.a().whileTrue(intake.getIntakeCmd(() -> Volts.of(-12)));
        // driverCtrl.b().whileTrue(intake.getIntakeCmd(() -> Volts.of(12 * 0.6)));

        driverCtrl.leftBumper().whileTrue(drivetrain.applyRequest(() -> drivetrain.getLookAtPointRequest(new Translation2d(Meters.of(11.913), Meters.of(4.041)))
            .withVelocityX(-driverCtrl.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverCtrl.getLeftX() * MaxSpeed)));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand(){
        SmartDashboard.putString("Current Command", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }


}

