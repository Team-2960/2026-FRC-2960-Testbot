package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldLayout;

public class ShooterHood extends SubsystemBase {

    // Motor
    private final TalonFX motor;
    private final CANcoder encoder;

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage posCtrl = new MotionMagicVoltage(0);

    private final CommandSwerveDrivetrain drivetrain;

    // SysId
    private final SysIdRoutine sysIdRoutime = new SysIdRoutine(
            new SysIdRoutine.Config(null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    null,
                    this));

    /**
     * Constructor
     * 
     * @param motorID
     */
    public ShooterHood(int motorId, int encoderId, CANBus bus, double gearRatio, CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        motor = new TalonFX(motorId, bus);
        encoder = new CANcoder(encoderId, bus);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

        motorConfig.Feedback
                .withSensorToMechanismRatio(1)
                .withRemoteCANcoder(encoder)
                .withRotorToSensorRatio(gearRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

        motorConfig.Slot0
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0);

        motor.getConfigurator().apply(motorConfig);
    }

    /**
     * Sets a target voltage to the motor
     * 
     * @param volts target voltage
     */
    public void setVoltage(Voltage volts) {
        motor.setControl(voltCtrl.withOutput(volts));
    }

    /**
     * Sets a target velocity to the motor
     * 
     * @param velocity target velocity
     */
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velCtrl.withVelocity(velocity));
    }

    /**
     * Sets a target velocity to the motor
     * 
     * @param velocity target velocity
     */
    public void setPosition(Angle angle) {
        motor.setControl(posCtrl.withPosition(angle));
    }

    /**
     * Gets the current voltage of the shooter hood
     * 
     * @return
     */
    @AutoLogOutput
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    /**
     * Gets the current velocity of the motor
     * 
     * @return current velocity of the motor
     */
    @AutoLogOutput
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    /**
     * Gets the current Position of the motor
     * 
     * @return current Position of the motor
     */
    @AutoLogOutput
    public Angle getPosition() {
        return motor.getPosition().getValue();
    }

    /**
     * Creates a new command to run the shooter hood at a set target voltage
     * 
     * @param target target voltage
     * @return new command to run the shooter hood at a set target voltage
     */
    public Command setVoltageCmd(Voltage target) {
        return this.runEnd(
                () -> setVoltage(target),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the shooter hood at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the shooter hood at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity target) {
        return this.runEnd(
                () -> setVelocity(target),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    /**
     * Creates a new command to run the shooter hood at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the shooter hood at a set target velocity
     */
    public Command setPositionCmd(Angle target) {
        return this.runEnd(
                () -> setPosition(target),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    /**
     * Creates a new command to run the shooter hood at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the shooter hood at a set target velocity
     */
    public Command setPositionCmd(Supplier<Angle> target) {
        return this.runEnd(
                () -> setPosition(target.get()),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    /**
     * Creates a new command to set the shooter hood for shooting at the hub
     * 
     * @return new command to set the shooter hood for shooting at the hub
     */
    public Command hubShotCmd() {
        return setPositionCmd(this::calcHubShotAngle);
    }

    /**
     * Create a Quasistatic SysId command
     * 
     * @param direction direction of the command
     * @return Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutime.quasistatic(direction);
    }

    /**
     * Create a Dynamic SysId command
     * 
     * @param direction direction of the command
     * @return Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutime.dynamic(direction);
    }

    /**
     * Update shooter hood RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Remove and use CTRE or AdvantageKit telemetry
        SmartDashboard.putNumber("Shooter Hood Angle", getPosition().in(Degrees));
        SmartDashboard.putNumber("Shooter Hood Angle RPM", getVelocity().in(Rotations.per(Minute)));
    }

    private Angle calcHubShotAngle() {
        Distance hubDist = FieldLayout.getHubDist(drivetrain.getPose2d().getTranslation());

        // TODO Implement Formula for target shooter hood angle

        return Rotations.zero();
    }
}
