/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class PIDElevator extends PIDSubsystem {

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Spark motor1;
  DigitalInput topLimit = null;
  DigitalInput bottomLimit = null;
  Encoder elevatorEncoder = null;
  double setPosition = 0;
  double currSpeed = 0;
  boolean useLimitSwitches = false;
  boolean useEncoders = false;

  // holds variables used to determine out of phase encoders
  private Robot.Direction dirMoved = Robot.Direction.NONE; 
  private double pastPosition = 0.0;
  
  public PIDElevator(){
    super("PIDElevator", RobotMap.ElevatorPID_P, RobotMap.ElevatorPID_I, RobotMap.ElevatorPID_D, RobotMap.ElevatorPID_F);

    motor1 = new Spark(RobotMap.SparkElevator);

    // enable the encoders and limit switches
    useLimitSwitches = true;
    useEncoders = true;

    // min and max output is -1 and 1 by default
    // looking at the code, if tolerance is absolute
    // then we don't need to set the input range
    setOutputRange(RobotMap.ElevatorMinOutput, RobotMap.ElevatorMaxOutput);

    if(useLimitSwitches){
      topLimit = new DigitalInput(RobotMap.ElevatorTopLimitSwitchId);
      bottomLimit = new DigitalInput(RobotMap.ElevatorBottomLimitSwitchId);
    }

    if(useEncoders){
      elevatorEncoder = new Encoder(RobotMap.ElevatorEncoderPIOId1, RobotMap.ElevatorEncoderPIOId2,false, Encoder.EncodingType.k2X);
      // initialize encoder
      elevatorEncoder.setDistancePerPulse(RobotMap.ElevatorDistPerPulse);
      elevatorEncoder.setReverseDirection(RobotMap.ElevatorReverseDirection);
    
      // reset needs to be run when the arm is at a known
      // angle/location,  Preferably this is a physical stop
      resetEncoder();
    }

    // initialize PID Controller
    setAbsoluteTolerance(RobotMap.ElevatorAbsTolerance);
    getPIDController().setContinuous(false);
    Robot.Log("PIDElevator Initialized");
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getSetPosition(){
    return setPosition;
  }

  public double getCurrentPosition(){
    if(useEncoders){
      return elevatorEncoder.getDistance();
    }
    else {
      return 0;
    }
  }

  public void setPIDPosition(double distance){
    if(! useEncoders){
      return;
    }

    Robot.Log("PIDElevator: setPosition:" + distance);
    setSetpoint(distance);
    setPosition = distance;

    // start the timing loop after we set the distance
    // this is needed for PID loop to start working
    getPIDController().enable();  
  }

  public void resetEncoder(){
    if(useEncoders){
      elevatorEncoder.reset();
    }
  }

  @Override
  public double returnPIDInput() { //originally protected
    double var = getCurrentPosition();
    //Robot.Log("PIDElevator: PIDInput: "+ var); 
    return var;  
  }

  @Override
  public void usePIDOutput(double output) {//originally protected
    //Robot.Log("PIDElevator: PIDOutput:" + output);
    motor1.pidWrite(output);
  }

  // used to manually set the speed which will disable pid control
  public void setSpeed(double speed){
    currSpeed = speed;
    getPIDController().disable();
    motor1.set(currSpeed);
  }

  public void Up() {
    dumpLimitSwitchValues();
    if (atUpperLimit()){
      Stop();
    } 
    else {
      VerifyEncoderPhase(pastPosition);
      dirMoved = Robot.Direction.UP;
      pastPosition = getCurrentPosition();
      setSpeed(RobotMap.ElevatorUpSpeed);
    }
  }
  
  public void Stop() {
    // or call stopmotor()?
    setSpeed(RobotMap.ElevatorStopSpeed);
  }
	
  public void Down() {
    dumpLimitSwitchValues();;
    if (atLowerLimit()){
      Stop();
    } 
    else {
     // Robot.Log("moving elevator down");
     VerifyEncoderPhase(pastPosition);
     dirMoved = Robot.Direction.DOWN;
     pastPosition = getCurrentPosition();
     setSpeed(RobotMap.ElevatorDownSpeed);
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

    // make sure the motor and encoder are in phase.  This means that
  // when we move the motor with a negative speed, the encoder
  // show we moved in the negative direction and vice versa
  private boolean VerifyEncoderPhase(double prevPos){
    double pos = getCurrentPosition();
    double deltaPos = pos - prevPos;
    double sign = 0;
    boolean check = true;
    boolean inPhase = true;

    switch(dirMoved){
      case DOWN:
        sign = Math.copySign(1, RobotMap.TalonWristDownSpeed);
        break;
      case UP:
        sign = Math.copySign(1, RobotMap.TalonWristUpSpeed);
        break;
      case NONE:
        check = false;
      break;
    }

    if( check  && (deltaPos != 0)){
      double deltaPosSign = Math.copySign(1, deltaPos);
      if( deltaPosSign != sign){
        inPhase = false;
        Robot.Log("Arm encoder is out of Phase from Arm Motor");
        Robot.die();
      }
    }
    return inPhase;
  }

}
