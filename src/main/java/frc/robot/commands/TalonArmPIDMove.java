/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.TalonEncodedArm;

public class TalonArmPIDMove extends Command {
  private double positionToMoveTo;

  public TalonArmPIDMove(double pos) {
    requires(Robot.encodedArmTalon);
    positionToMoveTo = pos;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.encodedArmTalon.setPIDPosition(positionToMoveTo);
    Robot.Log("Arm TalonPidMove: initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if the parameters are not set correctly, 
    // the arm position may never get to the targetPosition
    // and onTarget will never return true
    boolean done = Robot.encodedArmTalon.onTarget();
    Robot.Log("Arm TalonPidMove done:" + done);
    return done;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
