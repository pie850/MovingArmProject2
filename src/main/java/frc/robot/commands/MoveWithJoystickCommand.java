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
  
  //creates and initializes list of joystick positions
  private final List<Double> speedsArray = new ArrayList<>();

  /** Creates a new MoveWithJoystickCommand. */
  public MoveWithJoystickCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ExampleSubsystem.getInstance());
    m_subsystem = ExampleSubsystem.getInstance();
  }

  public void findAverage(double speed) {
    //change the name to something that is not "speed", it is very difficult to follow
    speedsArray.add(speed * 0.5);

    //maintain arclength for the derivative of following functions
    if (speedsArray.size() > 6) {
      speedsArray.remove(0);
    }
  
    double total = 0;
    for (double index : speedsArray) {
      total += index;
    }
    //find average
    double target_speed = total / speedsArray.size();

    m_subsystem.runMotor(target_speed);
  }

  public void setSpeed(double speed) {
    double overshoot;
    if (speed > 0) {
      if (m_subsystem.getEncoderDistance() >= 0.09) {
        findAverage(speed);
      } 
      else {
        if (m_subsystem.getEncoderDistance() >= 0.062) {
          overshoot = Math.abs(0.062 - m_subsystem.getEncoderDistance());
          System.out.println(overshoot*speed);
          findAverage(overshoot*speed);
        } else {
          Collections.fill(speedsArray, 0.0);
          findAverage(0.0);
        }
      }
    //ELSE IF BLOCK
    } else if (speed < 0) {
      if (m_subsystem.getEncoderDistance() <= 0.55) {
        findAverage(speed);
      }
      else {
        if (m_subsystem.getEncoderDistance() <= 0.58) {
          overshoot = Math.abs(0.58 - m_subsystem.getEncoderDistance());
          System.out.println(overshoot*speed);
          findAverage(overshoot*speed);
        } else {
          Collections.fill(speedsArray, 0.0);
          findAverage(0.0);
        }
      }
    //ELSE
    if (speed > 0 && m_subsystem.getEncoderDistance() >= 0.062) {
      findAverage(speed);
    } else if (speed < 0 && m_subsystem.getEncoderDistance() <= 0.569) {
      findAverage(speed);
    } else {
      findAverage(0.0);
    }
  }
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
  