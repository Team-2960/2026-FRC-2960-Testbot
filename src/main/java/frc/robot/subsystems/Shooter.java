package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;

public class Shooter extends SubsystemBase {
    private final DutyCycleOut shooterVolts = new DutyCycleOut(0.0);
    private final VoltageOut sysIDVolt = new VoltageOut(0.0);


    
    private final TalonFX shooterMotorL = new TalonFX(21, CANBus.roboRIO());
    private final TalonFX shooterMotorR = new TalonFX(20, CANBus.roboRIO());
    public Shooter(int shooterMotorLID, int shooterMotorRID) {

        shooterMotorR.setControl(new Follower(shooterMotorRID, MotorAlignmentValue.Opposed));
    }

    public void setControl(double output){

        shooterMotorL.setControl(shooterVolts.withOutput(output));
    }

    private final SysIdRoutine shooterSysIdRoutine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(null,
             Volts.of(4),
              null,
             (state) -> SignalLogger.writeString("state", state.toString()) 
            ), 
            new SysIdRoutine.Mechanism(
                (volts) -> shooterMotorR.setControl(sysIDVolt.withOutput(volts.in(Volts))),
         null,
         this
        )
    );
}

