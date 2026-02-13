package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.FieldLayout;

public class ShooterWheel extends SubsystemBase {
    // Motors
    private final TalonFX motorLeader;
    private final TalonFX motorFollower;

    // Motor Control Requests
    private final VoltageOut voltCtrl;
    private final MotionMagicVelocityVoltage velVoltCtrl;
    private final MotionMagicVelocityTorqueCurrentFOC velTorqueCtrl;

    private final CommandSwerveDrivetrain drivetrain;

    private Orchestra orchestra = new Orchestra();

    // SysId
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null,
                    Volts.of(4),
                    Seconds.of(5),
                    (state) -> SignalLogger.writeString("ShooterWheel", state.toString())),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    null,
                    this));

    private final SysIdRoutine sysIdRoutine2 = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(2), Seconds.of(5)),
            new SysIdRoutine.Mechanism(this::setVoltage, this::sysIDLogging, this));

    /**
     * Constructor
     * 
     * @param motorLeaderID   CAN ID of the lead shooter motor
     * @param motorFollowerID CAN ID of the follower shooter motor
     * @param bus             CAN Bus the shooter motors are on
     * @param gearRatio       Gear ratio between the shooter and the motor
     */
    public ShooterWheel(int motorLeaderID, int motorFollowerID, CANBus bus, double gearRatio,
            CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Initialize Motors
        motorLeader = new TalonFX(motorLeaderID, bus);
        motorFollower = new TalonFX(motorFollowerID, bus);

        // Configure Motors
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast);

        motorConfig.Feedback
                .withSensorToMechanismRatio(gearRatio);

        motorConfig.Slot0 // Voltage Control FF & PID
                .withKP(0.005)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.28683)
                .withKV(0.11886)
                .withKA(0.0067811);

        motorConfig.Slot1 // Torque FOC Control FF & PID
                .withKP(0.005)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.28683)
                .withKV(0)
                .withKA(0);

        motorConfig.MotionMagic
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(400))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Seconds).of(4000));

        motorLeader.getConfigurator().apply(motorConfig);

        motorFollower.setControl(new Follower(motorLeaderID, MotorAlignmentValue.Opposed));

        // Configure Requests
        voltCtrl = new VoltageOut(0.0)
                .withEnableFOC(true);

        velVoltCtrl = new MotionMagicVelocityVoltage(0)
                .withEnableFOC(true)
                .withAcceleration(Constants.shooterMaxAccel)
                .withSlot(0);

        velTorqueCtrl = new MotionMagicVelocityTorqueCurrentFOC(0)
                .withAcceleration(Constants.shooterMaxAccel)
                .withSlot(1);

        /**
         * motorLeader.getConfigurator().apply(new
         * AudioConfigs().withAllowMusicDurDisable(true));
         * motorFollower.getConfigurator().apply(new
         * AudioConfigs().withAllowMusicDurDisable(true));
         * 
         * orchestra.addInstrument(motorLeader);
         * orchestra.addInstrument(motorFollower);
         * orchestra.loadMusic("cry_for_me_ironmouse2.chrp");
         * //orchestra.play();
         */
    }

    /**
     * Gets the current voltage of the intake
     * 
     * @return
     */
    @AutoLogOutput
    public Voltage getVoltage() {
        return motorLeader.getMotorVoltage().getValue();
    }

    /**
     * Gets the current velocity of the motor
     * 
     * @return current velocity of the motor
     */
    @AutoLogOutput
    public AngularVelocity getVelocity() {
        return motorLeader.getVelocity().getValue();
    }

    /**
     * Gets the current velocity in RPM
     * 
     * @return current velocity in RPM
     */
    @AutoLogOutput
    public double getRPM() {
        return motorLeader.getVelocity().getValue().in(Rotations.per(Minute));
    }

    /**
     * Checks if the current velocity is within tolerance of the set point
     * 
     * @param tol measurement tolerance
     * @return true if in velocity control mode and within tolerance of the target.
     *         False otherwise
     */
    public boolean atVelocity(AngularVelocity tol) {
        boolean result = false;

        // Check if voltage velocity is running
        if (motorLeader.getAppliedControl() == velVoltCtrl) {
            result = MathUtil.isNear(
                    velVoltCtrl.Velocity,
                    getVelocity().in(RotationsPerSecond),
                    tol.in(RotationsPerSecond));
        }

        // Check if torque velocity
        if (motorLeader.getAppliedControl() == velTorqueCtrl) {
            result = MathUtil.isNear(
                    velTorqueCtrl.Velocity,
                    getVelocity().in(RotationsPerSecond),
                    tol.in(RotationsPerSecond));
        }

        return result;
    }

    /**
     * Sets the target motor voltage
     * 
     * @param volts target motor voltage
     */
    public void setVoltage(Voltage volts) {
        motorLeader.setControl(voltCtrl.withOutput(volts));
    }

    /**
     * Sets the target motor velocity. Voltage control is enabled by default.
     * 
     * @param velocity target motor velocity
     */
    public void setVelocity(AngularVelocity velocity) {
        setVelocity(velocity, false);
    }

    /**
     * Sets the target motor velocity
     * 
     * @param velocity   target motor velocity
     * @param torqueCtrl Set to true for torque control, false for voltage control
     */
    public void setVelocity(AngularVelocity velocity, boolean torqueCtrl) {
        ControlRequest request = torqueCtrl ? velTorqueCtrl.withVelocity(velocity) : velVoltCtrl.withVelocity(velocity);

        motorLeader.setControl(request);
    }

    /**
     * Creates a new command to run the intake at a set target voltage
     * 
     * @param volts target voltage
     * @return new command to run the intake at a set target voltage
     */
    public Command setVoltageCmd(Voltage input) {
        return this.runEnd(
                () -> setVoltage(input),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity. Defaults to voltage control.
     * 
     * @param volts target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity velocity) {
        return setVelocityCmd(velocity, false);
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param volts target velocity
     * @param torqueCtrl Set to true for torque control, false for voltage control
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity velocity, boolean torqueCtrl) {
        return this.runEnd(
                () -> setVelocity(velocity, torqueCtrl),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity. Defaults to voltage control.
     * 
     * @param volts target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(Supplier<AngularVelocity> velocity) {
        return setVelocityCmd(velocity, false);
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param volts target velocity
     * @param torqueCtrl Set to true for torque control, false for voltage control
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(Supplier<AngularVelocity> velocity, boolean torqueCtrl) {
        return this.runEnd(
                () -> setVelocity(velocity.get(), torqueCtrl),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to set the shooter for shooting at the hub
     * 
     * @return new command to set the shooter for shooting at the hub
     */
    public Command hubShotCmd() {
        return setVelocityCmd(this::calcHubShotSpeed);
    }

    /**
     * Create a Quasistatic SysId command
     * 
     * @param direction direction of the command
     * @return Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine2.quasistatic(direction);
    }

    /**
     * Create a Dynamic SysId command
     * 
     * @param direction direction of the command
     * @return Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine2.dynamic(direction);
    }

    /**
     * Update shooter RPM on dashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getVelocity().in(Rotations.per(Minute)));

        if (DriverStation.isEnabled()) {
            orchestra.stop();
        }
    }

    /**
     * Calculates the angular velocity for shooting into the hub from the current
     * robot position
     * 
     * @return target angular velocity
     */
    private AngularVelocity calcHubShotSpeed() {
        Distance hubDist = FieldLayout.getHubDist(drivetrain.getPose2d().getTranslation());

        // TODO Implement Formula for target shooter wheel speed

        return RotationsPerSecond.zero();
    }

    @AutoLogOutput
    public String getCommandString() {
        return this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName();
    }

    // Logs System ID
    private void sysIDLogging(SysIdRoutineLog log) {
        log.motor("ShooterWheel")
                .voltage(getVoltage())
                .current(motorLeader.getStatorCurrent().getValue())
                .angularVelocity(getVelocity())
                .angularPosition(motorLeader.getPosition().getValue());
    }
}
