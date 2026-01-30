package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    // Motors
    private final TalonFX motorLeader;
    private final TalonFX motorFollower;

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0);

    // SysId
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
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
     * @param motorLeaderID   CAN ID of the lead shooter motor
     * @param motorFollowerID CAN ID of the follower shooter motor
     * @param bus             CAN Bus the shooter motors are on
     * @param gearRatio       Gear ratio between the shooter and the motor
     */
    public Shooter(int motorLeaderID, int motorFollowerID, CANBus bus, double gearRatio) {
        motorLeader = new TalonFX(motorLeaderID, bus);
        motorFollower = new TalonFX(motorFollowerID, bus);

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

        motorLeader.getConfigurator().apply(motorConfig);

        motorLeader.getConfigurator().apply(motorConfig);
        motorFollower.setControl(new Follower(motorLeaderID, MotorAlignmentValue.Aligned));
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
     * Sets the target motor voltage
     * 
     * @param volts target motor voltage
     */
    public void setVoltage(Voltage volts) {
        motorLeader.setControl(voltCtrl.withOutput(volts));
    }

    /**
     * Sets the target motor velocity
     * 
     * @param velocity target motor velocity
     */
    public void setVelocity(AngularVelocity velocity) {
        motorLeader.setControl(velCtrl.withVelocity(velocity));
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
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param volts target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity velocity) {
        return this.runEnd(
                () -> setVelocity(velocity),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    /**
     * Create a Quasistatic SysId command
     * 
     * @param direction direction of the command
     * @return Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Create a Dynamic SysId command
     * 
     * @param direction direction of the command
     * @return Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Update shooter RPM on dashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getVelocity().in(Rotations.per(Minute)));
    }
}
