package frc.robot.Util.CustomSwerveRequests;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class FieldCentricCircularOrbit implements SwerveRequest{
    

    public double TravelVelocity = 0;

    public double Radius = 0;

    public Translation2d OrbitPoint = new Translation2d();

    public Rotation2d RotationOffset = new Rotation2d();

    public PIDController RadiusCorrectionPID = new PIDController(0, 0, 0);

    /**
     * The desired direction to face.
     * 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     */
    public Rotation2d TargetDirection = new Rotation2d();
    /**
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     */
    public double TargetRateFeedforward = 0;

    /**
     * The allowable deadband of the request, in m/s.
     */
    public double Deadband = 0;
    /**
     * The rotational deadband of the request, in radians per second.
     */
    public double RotationalDeadband = 0;
    /**
     * The maximum absolute rotational rate to allow, in radians per second.
     * Setting this to 0 results in no cap to rotational rate.
     */
    public double MaxAbsRotationalRate = 0;
    /**
     * The center of rotation the robot should rotate around.
     * This is (0,0) by default, which will rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying.
     * For more information, see the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /**
     * The perspective to use when determining which direction is forward.
     */
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    /**
     * The PID controller used to maintain the desired heading.
     * Users can specify the PID gains to change how aggressively to maintain
     * heading.
     * <p>
     * This PID controller operates on heading radians and outputs a target
     * rotational rate in radians per second. Note that continuous input should
     * be enabled on the range [-pi, pi].
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    private final FieldCentricFacingAngle m_fieldCentricFacingAngle = new FieldCentricFacingAngle();

    public FieldCentricCircularOrbit(){
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }


    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Pose2d curPose = parameters.currentPose;
        Rotation2d calcAngle = curPose.getTranslation().minus(OrbitPoint).getAngle().plus(RotationOffset);

        Translation2d targetPoint = OrbitPoint.plus(new Translation2d(Radius * calcAngle.getCos(), Radius * calcAngle.getSin()));

        double radiusCorrectionVelX = RadiusCorrectionPID.calculate(curPose.getX(), targetPoint.getX());
        double radiusCorrectionVelY = RadiusCorrectionPID.calculate(curPose.getY(), targetPoint.getY());

        double travelVelX = calcAngle.minus(Rotation2d.fromDegrees(90)).getCos() * TravelVelocity;
        double travelVelY = calcAngle.minus(Rotation2d.fromDegrees(90)).getSin() * TravelVelocity;

        double calcVelX = travelVelX + radiusCorrectionVelX;
        double calcVelY = travelVelY + radiusCorrectionVelY;

        System.out.println(calcAngle.getDegrees());

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
        .withTargetDirection(calcAngle)
        .withTargetRateFeedforward(TargetRateFeedforward)
        .withVelocityX(calcVelX)
        .withVelocityY(calcVelY)
        .apply(parameters, modulesToApply);
    }

    /**
         * Modifies the PID gains of the HeadingController parameter and returns itself.
         * <p>
         * Sets the proportional, integral, and differential coefficients used to maintain
         * the desired heading. Users can specify the PID gains to change how aggressively to
         * maintain heading.
         * <p>
         * This PID controller operates on heading radians and outputs a target
         * rotational rate in radians per second.
         *
         * @param kP The proportional coefficient; must be >= 0
         * @param kI The integral coefficient; must be >= 0
         * @param kD The differential coefficient; must be >= 0
         * @return this object
         */
        public FieldCentricCircularOrbit withHeadingPID(double kP, double kI, double kD)
        {
            this.HeadingController.setPID(kP, kI, kD);
            return this;
        }

        public FieldCentricCircularOrbit withTravelVelocity(double newVelocity){
            this.TravelVelocity = newVelocity;
            return this;
        }

        public FieldCentricCircularOrbit withTravelVelocity(LinearVelocity newVelocity){
            this.TravelVelocity = newVelocity.in(MetersPerSecond);
            return this;
        }

        public FieldCentricCircularOrbit withRadius(double radius){
            this.Radius = radius;
            return this;
        }
        
        public FieldCentricCircularOrbit withRadius(Distance radius){
            this.Radius = radius.in(Meters);
            return this;
        }

        public FieldCentricCircularOrbit withOrbitPoint(Translation2d point){
            this.OrbitPoint = point;
            return this;
        }

        public FieldCentricCircularOrbit withOrbitPoint(Distance x, Distance y){
            this.OrbitPoint = new Translation2d(x, y);
            return this;
        }

        public FieldCentricCircularOrbit withOrbitPoint(double x, double y){
            this.OrbitPoint = new Translation2d(x, y);
            return this;
        }

        public FieldCentricCircularOrbit withRotationalOffset(Rotation2d offset){
            this.RotationOffset = offset;
            return this;
        }

        public FieldCentricCircularOrbit withRotationalOffset(Angle offset){
            this.RotationOffset = new Rotation2d(offset);
            return this;
        }

        public FieldCentricCircularOrbit withRotationalOffset(double offset){
            this.RotationOffset = new Rotation2d(offset);
            return this;
        }

        public FieldCentricCircularOrbit withRadiusCorrectionPID(double kP, double kI, double kD){
            RadiusCorrectionPID.setPID(kP, kI, kD);
            return this;
        }

        /**
         * Modifies the TargetDirection parameter and returns itself.
         * <p>
         * The desired direction to face. 0 Degrees is defined as in the direction of
         * the X axis. As a result, a TargetDirection of 90 degrees will point along
         * the Y axis, or to the left.
         *
         * @param newTargetDirection Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withTargetDirection(Rotation2d newTargetDirection) {
            this.TargetDirection = newTargetDirection;
            return this;
        }

        /**
         * Modifies the TargetRateFeedforward parameter and returns itself.
         * <p>
         * The rotational rate feedforward to add to the output of the heading
         * controller, in radians per second. When using a motion profile for the
         * target direction, this can be set to the current velocity reference of
         * the profile.
         *
         * @param newTargetRateFeedforward Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withTargetRateFeedforward(double newTargetRateFeedforward) {
            this.TargetRateFeedforward = newTargetRateFeedforward;
            return this;
        }
        /**
         * Modifies the TargetRateFeedforward parameter and returns itself.
         * <p>
         * The rotational rate feedforward to add to the output of the heading
         * controller, in radians per second. When using a motion profile for the
         * target direction, this can be set to the current velocity reference of
         * the profile.
         *
         * @param newTargetRateFeedforward Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withTargetRateFeedforward(AngularVelocity newTargetRateFeedforward) {
            this.TargetRateFeedforward = newTargetRateFeedforward.in(RadiansPerSecond);
            return this;
        }

        /**
         * Modifies the Deadband parameter and returns itself.
         * <p>
         * The allowable deadband of the request, in m/s.
         *
         * @param newDeadband Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withDeadband(double newDeadband) {
            this.Deadband = newDeadband;
            return this;
        }

        /**
         * Modifies the Deadband parameter and returns itself.
         * <p>
         * The allowable deadband of the request, in m/s.
         *
         * @param newDeadband Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withDeadband(LinearVelocity newDeadband) {
            this.Deadband = newDeadband.in(MetersPerSecond);
            return this;
        }

        /**
         * Modifies the RotationalDeadband parameter and returns itself.
         * <p>
         * The rotational deadband of the request, in radians per second.
         *
         * @param newRotationalDeadband Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withRotationalDeadband(double newRotationalDeadband) {
            this.RotationalDeadband = newRotationalDeadband;
            return this;
        }

        /**
         * Modifies the RotationalDeadband parameter and returns itself.
         * <p>
         * The rotational deadband of the request, in radians per second.
         *
         * @param newRotationalDeadband Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withRotationalDeadband(AngularVelocity newRotationalDeadband) {
            this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
            return this;
        }

        /**
         * Modifies the MaxAbsRotationalRate parameter and returns itself.
         * <p>
         * The maximum absolute rotational rate to allow, in radians per second.
         * Setting this to 0 results in no cap to rotational rate.
         *
         * @param newMaxAbsRotationalRate Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withMaxAbsRotationalRate(double newMaxAbsRotationalRate) {
            this.MaxAbsRotationalRate = newMaxAbsRotationalRate;
            return this;
        }

        /**
         * Modifies the MaxAbsRotationalRate parameter and returns itself.
         * <p>
         * The maximum absolute rotational rate to allow, in radians per second.
         * Setting this to 0 results in no cap to rotational rate.
         *
         * @param newMaxAbsRotationalRate Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withMaxAbsRotationalRate(AngularVelocity newMaxAbsRotationalRate) {
            this.MaxAbsRotationalRate = newMaxAbsRotationalRate.in(RadiansPerSecond);
            return this;
        }

        /**
         * Modifies the CenterOfRotation parameter and returns itself.
         * <p>
         * The center of rotation the robot should rotate around. This is (0,0) by
         * default, which will rotate around the center of the robot.
         *
         * @param newCenterOfRotation Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withCenterOfRotation(Translation2d newCenterOfRotation) {
            this.CenterOfRotation = newCenterOfRotation;
            return this;
        }

        /**
         * Modifies the DriveRequestType parameter and returns itself.
         * <p>
         * The type of control request to use for the drive motor.
         *
         * @param newDriveRequestType Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
            this.DriveRequestType = newDriveRequestType;
            return this;
        }

        /**
         * Modifies the SteerRequestType parameter and returns itself.
         * <p>
         * The type of control request to use for the drive motor.
         *
         * @param newSteerRequestType Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
            this.SteerRequestType = newSteerRequestType;
            return this;
        }

        /**
         * Modifies the DesaturateWheelSpeeds parameter and returns itself.
         * <p>
         * Whether to desaturate wheel speeds before applying. For more information, see
         * the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
         *
         * @param newDesaturateWheelSpeeds Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
            this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
            return this;
        }

        /**
         * Modifies the ForwardPerspective parameter and returns itself.
         * <p>
         * The perspective to use when determining which direction is forward.
         *
         * @param newForwardPerspective Parameter to modify
         * @return this object
         */
        public FieldCentricCircularOrbit withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
            this.ForwardPerspective = newForwardPerspective;
            return this;
        }
}
