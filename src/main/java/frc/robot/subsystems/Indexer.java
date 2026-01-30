package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final DutyCycleOut indexerVolts = new DutyCycleOut(0.0);
    private final TalonFX indexMotor = new TalonFX(Constants.IndexMotorID, CANBus.roboRIO());
    private final VoltageOut sysIDVolt = new VoltageOut(0.0);
    
    final MotionMagicVelocityVoltage indexMotorMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

    public Indexer(int indexMotorId) {
        var indexMotorConfig = new Slot0Configs();
        indexMotorConfig.kP = 0.0;
        indexMotorConfig.kI = 0.0;
        indexMotorConfig.kD = 0.0;
        indexMotorConfig.kS = 0.0;
        indexMotorConfig.kV = 0.0;
        indexMotorConfig.kA = 0.0;

        indexMotor.getConfigurator().apply(indexMotorConfig);
    }

    public void setControl(double output){
        indexMotor.setControl(indexerVolts.withOutput(output));
    }

    public void setVelocity(double velocity){
        indexMotor.setControl(indexMotorMagicVelocityVoltage.withVelocity(velocity));
    }

    public Command getIndexCmd(double input) {
        return this.runEnd(
           () -> setControl(input),
           () -> setControl(0.0)
        );
    }

    public Command setVelocityCmd(double velocity) {
        return this.runEnd(
         () -> setVelocity(velocity),
         () -> setVelocity(0.0)
        );
    }
    private final SysIdRoutine indexSysIdRoutine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(null,
             Volts.of(4),
              null,
             (state) -> SignalLogger.writeString("state", state.toString()) 
            ), 
            new SysIdRoutine.Mechanism(
                (volts) -> indexMotor.setControl(sysIDVolt.withOutput(volts.in(Volts))),
         null,
         this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return indexSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return indexSysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Index RPM", indexMotor.getVelocity().getValue().in(Rotations.per(Minute)));
    }
}