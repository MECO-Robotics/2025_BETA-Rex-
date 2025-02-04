package frc.robot.subsystems.coral_intake;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CoralIntake extends SubsystemBase {
  // private SparkMax intakeMotor = new
  // SparkMax(CoralIntakeConstants.intakeCANId.intakeMotor, MotorType.kBrushless);
  // private SparkMax rotationMotor = new
  // SparkMax(CoralIntakeConstants.intakeCANId.rotationMotor,
  // MotorType.kBrushless);

  // private double initialIntakePosiiion;

  // public CoralIntake() {
  // intakeMotor.configure(null, null, null);
  // rotationMotor.configure(null, null, null);
  // }

  // public void setCoralIntakeVoltage(double voltage) {
  // intakeMotor.set(voltage);
  // }

  // public void setRotationMotorVoltage(double voltage) {
  // rotationMotor.set(voltage);
  // }

  // public void setIntakeMotorSpeed(double speed) {
  // intakeMotor.set(speed);
  // }

  // public void setRotationMotorSpeed(double speed) {
  // rotationMotor.set(speed);
  // }

}
