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

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private DutyCycleOut intakeVolts;

    public Intake(int intakeMotorID) {
        intakeMotor = new TalonFX(intakeMotorID, CANBus.roboRIO());
    }

    public void setVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage.in(Volts));
    }

    public Command getIntakeCmd(Supplier<Voltage> voltage) {
        return this.runEnd(
            () -> setVoltage(voltage.get()),
            () -> setVoltage(Volts.zero())
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake RPM", intakeMotor.getVelocity().getValue().in(Rotations.per(Minute)));
    }
}
