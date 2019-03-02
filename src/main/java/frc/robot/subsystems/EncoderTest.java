/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class EncoderTest extends Subsystem {
  Encoder encoder = null;

  public EncoderTest(){
    encoder = new Encoder(RobotMap.EncoderPIOId1, RobotMap.EncoderPIOId2,false, Encoder.EncodingType.k4X);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void resetEncoder(){
    encoder.reset();
  }

  public void LogValues(){
    int val = encoder.get();
    boolean valS = encoder.getStopped();
    double valR = encoder.getRaw();
    double valD = encoder.getDistance();
    Robot.Log("encoder: get:" + val + " stopped:" + valS + " raw:" + valR +  " dist:" + valD);
  }
}
