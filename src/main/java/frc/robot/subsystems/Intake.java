package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final DutyCycleOut intakeVolts = new DutyCycleOut(0.0);
    private final TalonFX intakeMotor = new TalonFX(Constants.IntakeMotorID, CANBus.roboRIO());
    private final VoltageOut sysIDVolt = new VoltageOut(0.0);
    
    final MotionMagicVelocityVoltage intakeMotorMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

    public Intake(int intakeMotorId) {
        var intakeMotorConfig = new Slot0Configs();
        intakeMotorConfig.kP = 0.0;
        intakeMotorConfig.kI = 0.0;
        intakeMotorConfig.kD = 0.0;
        intakeMotorConfig.kS = 0.0;
        intakeMotorConfig.kV = 0.0;
        intakeMotorConfig.kA = 0.0;

        intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    public void setControl(double output){
        intakeMotor.setControl(intakeVolts.withOutput(output));
    }

    public void setVelocity(double velocity){
        intakeMotor.setControl(intakeMotorMagicVelocityVoltage.withVelocity(velocity));
    }

    public Command getIntakeCmd(double input) {
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
