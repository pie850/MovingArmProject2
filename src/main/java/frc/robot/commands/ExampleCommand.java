// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;

  // private final PIDController pidController = new PIDController(15, 0.1, 0.4);
  private final PIDController pidController = new PIDController(3, 0.1, 0.1);
  private final PIDController accelerationPidController = new PIDController(0.02, 0, 0.01);
  private final double maxSpeed = 1.0;

  private double movePosition;
  private Timer timer;

  private final double targetAcceleration = 1;
  private double acceleration;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    movePosition = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    acceleration = 0.1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("start");
    Timer timer = new Timer();
    this.timer = timer;
    this.timer.start();
  }

  public void toPosition() {
    System.out.println("ran");
    double distance = movePosition - m_subsystem.getEncoderDistance();
    double pidCalculation = MathUtil.clamp(pidController.calculate(distance), -maxSpeed, maxSpeed);

    double accelerationDifference = targetAcceleration - acceleration;
    double accelerationCaculation = Math.abs(MathUtil.clamp(accelerationPidController.calculate(accelerationDifference), -1, 1));
    System.out.println("Acceleration calculation: " + accelerationCaculation);
    acceleration = MathUtil.clamp(acceleration + accelerationCaculation, 0, 1);
    
    System.out.println(MathUtil.clamp(pidCalculation * acceleration, -1, 1));

    m_subsystem.setMotor(MathUtil.clamp(pidCalculation * acceleration, -1, 1));
    // m_subsystem.setMotor(MathUtil.clamp(pidCalculation, -1, 1));
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Acceleration: " + acceleration);
    toPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end");
    m_subsystem.setMotor(0.0);
    acceleration = 0.1;
    
    // System.out.println(timer.get());
    // System.out.println(m_subsystem.getEncoderDistance());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(movePosition - m_subsystem.getEncoderDistance()) <= 0.01;
  }
}
