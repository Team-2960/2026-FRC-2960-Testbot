package frc.robot.Util.CustomSwerveRequests;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldCentricGoToPoint implements SwerveRequest {   
    
    /**
     * The desired velocity to travel along the circle created around the orbital point using the radius.
     * The travel velocity is eventually split into an X and Y Velocity to feed into the FieldCentric Request.
     */
    public double TravelVelocity = 0;

    /**
     * The target point for the robot to move to.
     */
    public Translation2d TargetPoint = new Translation2d();

    /**
     * The PID Controller used to calculate the translation speed toward the point.
     * kP should be tuned so the robot slows down as it approaches the target.
     */
    public PIDController TranslationPID = new PIDController(0, 0, 0);

    /**
     * The desired direction to face while moving to the point.
     */
    public Rotation2d TargetDirection = new Rotation2d();

    /**
     * Offset applied to the rotation.
     */
    public Rotation2d RotationOffset = new Rotation2d();

    public double TargetRateFeedforward = 0;
    public double Deadband = 0;
    public double RotationalDeadband = 0;
    public double MaxAbsRotationalRate = 0;
    public Translation2d CenterOfRotation = new Translation2d();

    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    public boolean DesaturateWheelSpeeds = true;
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    /**
     * The PID controller used to maintain the desired heading.
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    private final FieldCentricFacingAngle m_fieldCentricFacingAngle = new FieldCentricFacingAngle();
        public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public FieldCentricGoToPoint() {
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Pose2d robotPose = parameters.currentPose;

        // 1. Calculate vector to target
        Translation2d relativeVector = TargetPoint.minus(robotPose.getTranslation());
        double distanceToTarget = relativeVector.getNorm();

        // 2. Prevent division by zero and handle arrival deadband
        double vx = 0;
        double vy = 0;

        if (distanceToTarget > 0.01) { // 1cm tolerance to avoid NaN
            // Calculate speed toward the point using PID
            // The setpoint is 0 (we want distance to target to be 0)
            double translationMag = Math.abs(TranslationPID.calculate(distanceToTarget, 0));
            if (isRedAlliance()){
                translationMag = -translationMag;
            }
            
            
            // Limit to TravelVelocity
            if (Math.abs(translationMag) > TravelVelocity) {
                translationMag = Math.copySign(TravelVelocity, translationMag);
            }

            // Convert distance vector into a unit vector and multiply by calculated velocity
            Translation2d unitVector = relativeVector.div(distanceToTarget);
            vx = unitVector.getX() * translationMag;
            vy = unitVector.getY() * translationMag;
        }

        // 3. Apply to the underlying FacingAngle request
        return m_fieldCentricFacingAngle
                .withCenterOfRotation(CenterOfRotation)
                .withDeadband(Deadband)
                .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
                .withDriveRequestType(DriveRequestType)
                .withForwardPerspective(ForwardPerspective)
                .withHeadingPID(HeadingController.getP(), HeadingController.getI(), HeadingController.getD())
                .withMaxAbsRotationalRate(MaxAbsRotationalRate)
                .withRotationalDeadband(RotationalDeadband)
                .withSteerRequestType(SteerRequestType)
                .withTargetDirection(TargetDirection.plus(RotationOffset))
                .withTargetRateFeedforward(TargetRateFeedforward)
                .withVelocityX(vx)
                .withVelocityY(vy)
                .apply(parameters, modulesToApply);
    }

    /* Builder Methods */

    public FieldCentricGoToPoint withTargetPoint(Translation2d point) {
        this.TargetPoint = point;
        return this;
    }

    public FieldCentricGoToPoint withTranslationPID(double kP, double kI, double kD) {
        this.TranslationPID.setPID(kP, kI, kD);
        return this;
    }

    public FieldCentricGoToPoint withTravelVelocity(double velocity) {
        this.TravelVelocity = velocity;
        return this;
    }

    public FieldCentricGoToPoint withTravelVelocity(LinearVelocity velocity) {
        this.TravelVelocity = velocity.in(MetersPerSecond);
        return this;
    }

    public FieldCentricGoToPoint withHeadingPID(double kP, double kI, double kD) {
        this.HeadingController.setPID(kP, kI, kD);
        return this;
    }

    public FieldCentricGoToPoint withTargetDirection(Rotation2d direction) {
        this.TargetDirection = direction;
        return this;
    }

    public FieldCentricGoToPoint withRotationalOffset(Rotation2d offset) {
        this.RotationOffset = offset;
        return this;
    }

    // ... (Keep existing withDeadband, withForwardPerspective, etc. from your original class)
}