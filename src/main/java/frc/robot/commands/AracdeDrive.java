// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AracdeDrive extends CommandBase {
  public DriveTrain _DT;
  public Joystick _cow;
  /** Creates a new AracdeDrive. */
  public AracdeDrive(DriveTrain dt, Joystick js) {
    // Use addRequirements() here to declare subsystem dependencies.
    _DT = dt; 
    _cow = js;
    addRequirements(_DT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -0.7*_cow.getRawAxis(Constants.JoystickAxis.YAxis);
    double turn = 0.7 *_cow.getRawAxis(Constants.JoystickAxis.XAxis);
    _DT.arcadeDrive(speed, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}