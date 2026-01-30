package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;


import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

public class Intake extends SubsystemBase {
    private final VoltageOut intakeVolts = new VoltageOut(0.0);
    private final TalonFX intakeMotor = new TalonFX(Constants.IntakeMotorID, CANBus.roboRIO());
    private final VoltageOut sysIDVolt = new VoltageOut(0.0);
    
    private final MotionMagicVelocityVoltage intakeMotorMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);


    public Intake(int intakeMotorId) {
        MotorOutputConfigs outputConfig = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        var closedLoopConfig = new Slot0Configs();
        closedLoopConfig.kP = 0.0;
        closedLoopConfig.kI = 0.0;
        closedLoopConfig.kD = 0.0;
        closedLoopConfig.kS = 0.0;
        closedLoopConfig.kV = 0.0;
        closedLoopConfig.kA = 0.0;

        intakeMotor.getConfigurator().apply(closedLoopConfig);
        intakeMotor.getConfigurator().apply(outputConfig);
    }

    
    public void setVoltage(Voltage volts){
        intakeMotor.setControl(intakeVolts.withOutput(volts));
    }

    public void setVelocity(AngularVelocity velocity){
        intakeMotor.setControl(intakeMotorMagicVelocityVoltage.withVelocity(velocity));
    }

    @AutoLogOutput
    public Voltage getVoltage(){
        return intakeMotor.getMotorVoltage().getValue();
    }
    
    @AutoLogOutput
    public AngularVelocity getVelocity(){
        return intakeMotor.getVelocity().getValue();
    }

    public Command setVoltageCmd(Voltage volts) {
        return this.runEnd(
           () -> setVoltage(volts),
           () -> setVoltage(Volts.zero())
        );
    }

    public Command setVelocityCmd(AngularVelocity velocity) {
        return this.runEnd(
         () -> setVelocity(velocity),
         () -> setVelocity(RotationsPerSecond.zero())
        );
    }

    private final SysIdRoutine intakeSysIdRoutine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(null,
             Volts.of(4),
              null,
             (state) -> SignalLogger.writeString("state", state.toString()) 
            ), 
            new SysIdRoutine.Mechanism(
                (volts) -> intakeMotor.setControl(sysIDVolt.withOutput(volts.in(Volts))),
         null,
         this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return intakeSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return intakeSysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake RPM", intakeMotor.getVelocity().getValue().in(Rotations.per(Minute)));
    }
}
