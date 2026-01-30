package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Util.AprilTagPipelineSettings;
import frc.robot.generated.TunerConstants;

public class Constants {
    // Robot Constants
    public static final LinearVelocity maxLinVel = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity maxAngVel = RotationsPerSecond.of(2); 
    public static final LinearVelocity slowdownLinVel = maxLinVel.div(2);
    public static final AngularVelocity slowdownAngVel = RotationsPerSecond.of(.5);

    public static final CANBus rioBus = CANBus.roboRIO();
    public static final CANBus canivoreBus = new CANBus("canivore");

    // Motor IDs
    public static final int shooterMotorLID = 999;
    public static final int shooterMotorRID = 999;
    public static final int IntakeMotorID = 19;
    public static final int IndexMotorID = 999;

    // Shooter Constants
    public static final double ShooterShootingVelocity = 0;
    public static final double IndexerShootingVelocity = 0;
    public static final double ShooterChargeVelocity = 0;
    public static final double IndexerChargeVelocity = 0;

    // Intake Constants
    public static final double intakeGearRatio = 1;

    public static final Voltage intakeInVolt = Volts.of(12.0);
    public static final Voltage intakeOutVolt = Volts.of(-12.0);

    // Drivetrain Constants
    public static final LinearVelocity linDeadband = maxLinVel.times(.1); 
    public static final AngularVelocity angDeadband = maxAngVel.times(.1);

    // Camera Constants
    public static final Transform3d leftCameraOffsets = new Transform3d(
        Inches.of(-29/2).plus(Millimeters.of(37.492)), 
        Inches.of(29/2).minus(Millimeters.of(55.321)), 
        Inches.of(5.375), 
        new Rotation3d(0, Math.toRadians(-30), Math.toRadians(-135))
    );

    public static final Transform3d rightCameraOffsets = new Transform3d(
        Inches.of(-29/2).plus(Millimeters.of(37.492)), 
        Inches.of(-29/2).plus(Millimeters.of(55.321)), 
        Inches.of(5.375), 
        //new Rotation3d(37.8, 37.8, -45)
        new Rotation3d(0, Math.toRadians(-30), Math.toRadians(135))
    );

    public static final Vector<N3> singleStds = VecBuilder.fill(4, 4, 16);
    public static final Vector<N3> multiStds = VecBuilder.fill(0.5, 0.5, 1);

    public static final AprilTagPipelineSettings leftCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.leftCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        3, 
        singleStds , 
        multiStds,
        .2
    );

    public static final AprilTagPipelineSettings rightCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.rightCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        3, 
        singleStds , 
        multiStds,
        .2
    );

}
