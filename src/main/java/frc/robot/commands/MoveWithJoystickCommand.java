// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWithJoystickCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;

  int count = 0;
  final int MAX_COUNT = 6;
  
  //creates and initializes list of joystick positions
  private final List<Double> speedsArray = new ArrayList<>();

  /** Creates a new MoveWithJoystickCommand. */
  public MoveWithJoystickCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ExampleSubsystem.getInstance());
    m_subsystem = ExampleSubsystem.getInstance();
  }

  public void setSpeed(double speed) {
      if (speed == 0) {
          count = 0;
      }
      if (count < MAX_COUNT) {
          count++;
          speed = speed * (count / MAX_COUNT);
      }
      m_subsystem.runMotor(speed);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
