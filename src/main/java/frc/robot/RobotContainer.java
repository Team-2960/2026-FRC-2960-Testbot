// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagPipeline;
import frc.robot.subsystems.CameraSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(Constants.maxLinVel.in(MetersPerSecond));

    private final CommandXboxController driverCtrl = new CommandXboxController(0);
    @SuppressWarnings("unused")
    private final CommandXboxController operatorCtrl = new CommandXboxController(1);

    // Physical Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Intake intake = new Intake(Constants.IntakeMotorID);
    // private final Indexer indexer = new Indexer(Constants.IndexMotorID);
    // private final Shooter shooter = new Shooter(Constants.shooterMotorLID,
    // Constants.shooterMotorRID);

    // Cameras
    private final AprilTagPipeline leftCamera = new AprilTagPipeline(drivetrain, Constants.leftCameraSettings,
            "LeftCamera", "LeftCamera");
    private final AprilTagPipeline rightCamera = new AprilTagPipeline(drivetrain, Constants.rightCameraSettings,
            "RightCamera", "RightCamera");
    
    @SuppressWarnings("unused")
    private final CameraSim cameraSim = new CameraSim(drivetrain, leftCamera, rightCamera);

    // Pathplanner
    SendableChooser<Command> autoChooser;

    // Mutable Units
    private MutLinearVelocity xVel = MetersPerSecond.mutable(0);
    private MutLinearVelocity yVel = MetersPerSecond.mutable(0);
    private MutAngularVelocity rVel = RotationsPerSecond.mutable(0);

    // Triggers
    private Trigger testMode = new Trigger(DriverStation::isTest);

    // Standard Suppliers
    private Supplier<LinearVelocity> fullXVelCtrl = () -> xVel.mut_replace(Constants.maxLinVel)
            .mut_times(-driverCtrl.getLeftY());
    private Supplier<LinearVelocity> fullYVelCtrl = () -> yVel.mut_replace(Constants.maxLinVel)
            .mut_times(-driverCtrl.getLeftX());
    private Supplier<AngularVelocity> fullRVelCtrl = () -> rVel.mut_replace(Constants.maxAngVel)
            .mut_times(-driverCtrl.getRightX());

    private Supplier<LinearVelocity> slowXVelCtrl = () -> xVel.mut_replace(Constants.slowdownLinVel)
            .mut_times(-driverCtrl.getLeftY());
    private Supplier<LinearVelocity> slowYVelCtrl = () -> yVel.mut_replace(Constants.slowdownLinVel)
            .mut_times(-driverCtrl.getLeftX());
    private Supplier<AngularVelocity> slowRVelCtrl = () -> rVel.mut_replace(Constants.slowdownAngVel)
            .mut_times(-driverCtrl.getRightX());

    /**
     * Constructor
     */
    public RobotContainer() {
        // Init PathPlanner
        initPathPlanner();

        // Configure Robot Bindings
        configureBindings();

        // Initialize drivetrain telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Initializes PathPlanner
     */
    private void initPathPlanner() {

        NamedCommands.registerCommand("Intake Command", intake.setVoltageCmd(Constants.intakeInVolt));

        // Initialize Auton chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);
    }

    /**
     * Configures robot bindings
     */
    private void configureBindings() {

        drivetrainBindings();
        intakeBindings();
        
    }

    /**
     * Configures drivetrain bindings
     */
    private void drivetrainBindings() {
        // Set default drivetrain command
        drivetrain.setDefaultCommand(
                drivetrain.getDriveCmd(
                        fullXVelCtrl,
                        fullYVelCtrl,
                        fullRVelCtrl));

        // Slow Drive Command
        driverCtrl.rightBumper().whileTrue(
                drivetrain.getDriveCmd(
                        slowXVelCtrl,
                        slowYVelCtrl,
                        slowRVelCtrl));

        // Track Goal
        driverCtrl.leftBumper().whileTrue(
                drivetrain.lookAtPointCmd(
                        fullXVelCtrl,
                        fullYVelCtrl,
                        FieldLayout.getHubCenter(),
                        Rotation2d.fromDegrees(180)));

        // Pose Reset
        driverCtrl.pov(0).onTrue(drivetrain.runOnce(
                () -> drivetrain.resetPose(
                        new Pose2d(
                                FieldLayout.getHubCenterFront(),
                                Rotation2d.fromDegrees(FieldLayout.getForwardAngle().in(Degrees) + 180)))));

        // Idle motors when disabled
        RobotModeTriggers.disabled().whileTrue(drivetrain.idleCmd());

        // Drivetrain SysId Controls
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        testMode.and(driverCtrl.start()).and(driverCtrl.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        testMode.and(driverCtrl.start()).and(driverCtrl.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        testMode.and(driverCtrl.start()).and(driverCtrl.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        testMode.and(driverCtrl.start()).and(driverCtrl.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    private void intakeBindings() {
        driverCtrl.a().whileTrue(intake.setVoltageCmd(Constants.intakeOutVolt));
        driverCtrl.b().whileTrue(intake.setVoltageCmd(Constants.intakeInVolt));
    }

    /**
     * Retrieves the selected auton
     * 
     * @return the selected auton
     */
    public Command getAutonomousCommand() {
        SmartDashboard.putString("Current Command", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }

}
