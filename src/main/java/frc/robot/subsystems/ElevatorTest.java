/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class ElevatorTest extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DigitalInput topLimit = null;
  DigitalInput bottomLimit = null;
  Spark motor = null;
  int count = 0;

  public ElevatorTest(){
    motor = new Spark(RobotMap.SparkElevator);
    motor.enableDeadbandElimination(true);	
  }

  public void Up(){
    motor.set(RobotMap.ElevatorUpSpeed);
    if( (count % 10) == 0) {
     // Robot.encoderTest.LogValues();
    }
    count++;

  }
  public void Down(){
    motor.set(RobotMap.ElevatorDownSpeed);
    if( (count % 10) == 0) {
      //Robot.encoderTest.LogValues();
    }
    count++;
  }
  public void Stop(){
    motor.stopMotor();
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
