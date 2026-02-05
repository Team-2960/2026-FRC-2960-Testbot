// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.thethriftybot.server.CAN;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagPipeline;
import frc.robot.subsystems.CameraSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.ShooterWheel;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(Constants.maxLinVel.in(MetersPerSecond));

    private final CommandXboxController driverCtrl = new CommandXboxController(0);

    @SuppressWarnings("unused")
    private final CommandXboxController operatorCtrl = new CommandXboxController(1);

    // Physical Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final IntakeRoller intake = new IntakeRoller(
            Constants.IntakeMotorID,
            CANBus.roboRIO(),
            Constants.intakeGearRatio);

    private final ShooterWheel shooter = new ShooterWheel(31, 32, CANBus.roboRIO(), 1, drivetrain);
    private final Indexer indexer = new Indexer(21, null, 1);
        

//     // Cameras
    private final AprilTagPipeline leftCamera = new AprilTagPipeline(
            drivetrain,
            Constants.leftCameraSettings,
            "LeftCamera",
            "LeftCamera");
    private final AprilTagPipeline rightCamera = new AprilTagPipeline(
            drivetrain,
            Constants.rightCameraSettings,
            "RightCamera",
            "RightCamera");

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
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getLeftY(), 0.05));
    private Supplier<LinearVelocity> fullYVelCtrl = () -> yVel.mut_replace(Constants.maxLinVel)
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getLeftX(), 0.05));
    private Supplier<AngularVelocity> fullRVelCtrl = () -> rVel.mut_replace(Constants.maxAngVel)
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getRightX(), 0.05));

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
        shooterBindings();
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

        driverCtrl.a().whileTrue(
                drivetrain.hubOrbitCommand(fullYVelCtrl, Rotation2d.fromDegrees(180), Meters.of(3))
        );

        driverCtrl.b().whileTrue(
                drivetrain.hubOrbitRestrictedRadiusCommand(fullYVelCtrl, fullXVelCtrl, Rotation2d.fromDegrees(180), Meters.of(3), Meters.of(1.75))
        );

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
        // // Note that each routine should be run exactly once in a single log.
        // testMode.and(driverCtrl.start()).and(driverCtrl.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // testMode.and(driverCtrl.start()).and(driverCtrl.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // testMode.and(driverCtrl.start()).and(driverCtrl.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //testMode.and(driverCtrl.start()).and(driverCtrl.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // testMode.and(driverCtrl.start()).and(driverCtrl.x()).onTrue(shooter.sysIdQuasistatic(Direction.kForward));
        // testMode.and(driverCtrl.start()).and(driverCtrl.y()).onTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        // testMode.and(driverCtrl.start()).and(driverCtrl.a()).onTrue(shooter.sysIdDynamic(Direction.kForward));
        // testMode.and(driverCtrl.start()).and(driverCtrl.b()).onTrue(shooter.sysIdDynamic(Direction.kReverse));

        Command sysIdCommandGroup = Commands.sequence(
                shooter.sysIdQuasistatic(Direction.kForward),
                shooter.sysIdQuasistatic(Direction.kReverse),
                shooter.sysIdDynamic(Direction.kForward),
                shooter.sysIdDynamic(Direction.kReverse));

        driverCtrl.start().onTrue(sysIdCommandGroup);
    }

    private void intakeBindings() {
        operatorCtrl.leftTrigger(0.1).whileTrue(intake.setVoltageCmd(Constants.intakeOutVolt));
        operatorCtrl.leftBumper().whileTrue(intake.setVoltageCmd(Constants.intakeInVolt));
    }

    private void shooterBindings(){
        operatorCtrl.rightBumper().whileTrue(shooter.setVoltageCmd(Volts.of(4)));
        operatorCtrl.rightTrigger(.1).whileTrue(shooter.setVelocityCmd(Rotations.per(Minute).of(1500)));
        operatorCtrl.a().whileTrue(indexer.setVoltageCmd(Volts.of(-12)));
        operatorCtrl.b().whileTrue(indexer.setVoltageCmd(Volts.of(12)));
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
