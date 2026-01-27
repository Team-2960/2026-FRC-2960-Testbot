package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Indexer extends SubsystemBase {
    private final TalonFX indexMotor;
    private DutyCycleOut indexVolts;

    public Indexer(int indexMotorID) {
        indexMotor = new TalonFX(indexMotorID, CANBus.roboRIO());
    }

    public void setVoltage(Voltage voltage) {
        indexMotor.setVoltage(voltage.in(Volts));
    }

    public Command getIndexCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
            () -> setVoltage(voltage.get()),
            () -> setVoltage(Volts.zero())
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Index RPM", indexMotor.getVelocity().getValue().in(Rotations.per(Minute)));
    }
}