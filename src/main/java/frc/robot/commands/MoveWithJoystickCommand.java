// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveWithJoystickCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  
  //creates and initializes list of joystick positions
  private final List<Double> speedsArray = new ArrayList<>();

  PIDController velocityPID = new PIDController(0.1, 0.05, 0.3);

  /** Creates a new MoveWithJoystickCommand. */
  public MoveWithJoystickCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ExampleSubsystem.getInstance());
    m_subsystem = ExampleSubsystem.getInstance();
  }

  public void findAverage(double speed) {

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
    speed = speed - m_subsystem.getEncoderVelocity();
    speed = MathUtil.clamp(velocityPID.calculate(speed), -1, 1);

    findAverage(speed);

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
