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
  Encoder encoder = null;
  int count = 0;
  boolean useLimitSwitches = true;
  boolean useEncoders = true;

  public ElevatorTest(){
    motor = new Spark(RobotMap.SparkElevator);
    motor.enableDeadbandElimination(true);	
 
    if(useLimitSwitches){
      topLimit = new DigitalInput(RobotMap.ElevatorTopLimitSwitchId);
      bottomLimit = new DigitalInput(RobotMap.ElevatorBottomLimitSwitchId);
    }

    if(useEncoders){
      encoder = new Encoder(RobotMap.ElevatorEncoderPIOId1, RobotMap.ElevatorEncoderPIOId2,false, Encoder.EncodingType.k2X);
      // initialize encoder
      encoder.setDistancePerPulse(RobotMap.ElevatorDistPerPulse);
      encoder.setReverseDirection(RobotMap.ElevatorReverseDirection);
    
      // reset needs to be run when the arm is at a known
      // angle/location,  Preferably this is a physical stop
      resetEncoder();
    }

 }

 public void resetEncoder(){
  if(useEncoders){
    encoder.reset();
  }
}

public boolean atUpperLimit(){
  if(!useLimitSwitches){
    return false;
  }

  return topLimit.get();
}

public boolean atLowerLimit() {
  if(!useLimitSwitches){
    return false;
  }

  // currently the bottom limit switched is wired
  // differently so true is not set and false is set
  // so we need to invert the logic
  return ! bottomLimit.get();
}

public void dumpLimitSwitchValues(){
  if(!useLimitSwitches){
    return;
  }

  Robot.Log("Elevator: atLowerLimit:" + atLowerLimit() + " atUpperLimit:" + atUpperLimit());
}  

// used to manually set the speed which will disable pid control
public void setSpeed(double speed){
  motor.set(speed);
}

public void Up() {
  //dumpLimitSwitchValues();
  LogValues();
  if (atUpperLimit()){
    Stop();
  } 
  else {
    setSpeed(RobotMap.ElevatorUpSpeed);
  }
}

public void Stop() {
  // or call stopmotor()?
  setSpeed(RobotMap.ElevatorStopSpeed);
}

public void Down() {
  LogValues();
  //dumpLimitSwitchValues();;
  if (atLowerLimit()){
    Stop();
  } 
  else {
   setSpeed(RobotMap.ElevatorDownSpeed);
  }
}

public void LogValues(){
  int val = encoder.get();
  boolean valS = encoder.getStopped();
  double valR = encoder.getRaw();
  double valD = encoder.getDistance();
  Robot.Log("encoder: get:" + val + " stopped:" + valS + " raw:" + valR +  " dist:" + valD);
}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
