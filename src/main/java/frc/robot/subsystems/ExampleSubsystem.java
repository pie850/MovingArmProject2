// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ExampleSubsystem extends SubsystemBase {

  public static ExampleSubsystem m_Subsystem;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  public final SparkFlex m_Spark = new SparkFlex(1, MotorType.kBrushless);
  public final SparkAbsoluteEncoder m_Encoder = m_Spark.getAbsoluteEncoder();

  public static ExampleSubsystem getInstance() {
    if (m_Subsystem == null) {
      m_Subsystem = new ExampleSubsystem();
    }
    return m_Subsystem;
  }

  public double getEncoderDistance() {
    return m_Encoder.getPosition();
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    m_Spark.set(speed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
