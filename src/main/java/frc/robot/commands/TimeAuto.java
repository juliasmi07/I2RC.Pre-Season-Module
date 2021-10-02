// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TimeAuto extends CommandBase {
  Timer timedAuto = new Timer(); 
  private final DriveTrain dt;

  public TimeAuto(DriveTrain drive) {
    dt = drive;

    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timedAuto.start();
    timedAuto.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timedAuto.get() <= 3){
      dt.tankDrive(.8, .8);
    }
    
    if (timedAuto.get() > 5){
      dt.tankDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timedAuto.get() > 5;
  }
}
