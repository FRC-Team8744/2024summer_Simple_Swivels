// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class Outtake extends Command {
  private final Intake m_intake;
  private final Indexer m_index;
  /** Creates a new Outtake. */
  public Outtake(Intake intake, Indexer index) {
    m_intake = intake;
    m_index = index;
    addRequirements(m_intake);
    addRequirements(m_index);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.reverseIntake(Constants.MechanismConstants.intakeSpeed);
    m_index.reverseIndexer(Constants.MechanismConstants.indexerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.motorOff();
    m_index.motorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
