// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  
//hello

  private final PIDController pidController = new PIDController(15, 0.1, 0.4);
  private final double maxSpeed = 1.0;

  private double movePosition;
  private Timer timer;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem, double speed, double position) {
    m_subsystem = subsystem;
    movePosition = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Timer timer = new Timer();
    this.timer = timer;
    this.timer.start();
  }

  public void toPosition() {
    double distance = movePosition - m_subsystem.getEncoderDistance();
    double pidCalculation = MathUtil.clamp(pidController.calculate(distance), -maxSpeed, maxSpeed);
    m_subsystem.setMotor(pidCalculation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotor(0.0);
    
    System.out.println(timer.get());
    System.out.println(m_subsystem.getEncoderDistance());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(movePosition - m_subsystem.getEncoderDistance()) <= 0.01;
  }
}
