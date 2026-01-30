package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    // Motor
    private final TalonFX motor;

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0);

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
    public Indexer(int motorId, CANBus bus, double gearRatio) {
        motor = new TalonFX(motorId, bus);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

        motorConfig.Feedback
                .withSensorToMechanismRatio(gearRatio);

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
     * Gets the current voltage of the indexer
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
     * Creates a new command to run the indexer at a set target voltage
     * 
     * @param volts target voltage
     * @return new command to run the indexer at a set target voltage
     */
    public Command setVoltageCmd(Voltage volts) {
        return this.runEnd(
                () -> setVoltage(volts),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the indexer at a set target velocity
     * 
     * @param volts target velocity
     * @return new command to run the indexer at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity velocity) {
        return this.runEnd(
                () -> setVelocity(velocity),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    /**
     * Creates a command to feed the shooter
     * @return  command to feed the shooter
     */
    public Command runShooterFeed() {
        return setVoltageCmd(Constants.indexerFeedVolt);
    }

    /**
     * Creates a command to stop feeding the shooter
     * @return  command to stop feeding the shooter
     */
    public Command stopShooterFeed() {
        return setVoltageCmd(Volts.zero());
    }

    /**
     * Create a Quasistatic SysId command
     * @param direction direction of the command
     * @return  Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutime.quasistatic(direction);
    }


    /**
     * Create a Dynamic SysId command
     * @param direction direction of the command
     * @return  Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutime.dynamic(direction);
    }

    /**
     * Update indexer RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Remove and use CTRE or AdvantageKit telemetry
        SmartDashboard.putNumber("Indexer RPM", getVelocity().in(Rotations.per(Minute)));
    }
}
