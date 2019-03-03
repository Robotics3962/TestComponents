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
import frc.robot.commands.ElevatorHoldCmd;

/**
 * Add your docs here.
 */
public class PIDElevator extends PIDSubsystem {

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Spark motor1;
  private DigitalInput topLimit = null;
  private DigitalInput bottomLimit = null;
  private Encoder elevatorEncoder = null;
  private double targetPosition = 0;
  private double currSpeed = 0;
  private boolean useLimitSwitches = true;
  private boolean useEncoders = true;
  private boolean manualOverride = false;

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
    setDefaultCommand(new ElevatorHoldCmd());
  }

  public double getSetPosition(){
    return targetPosition;
  }

  public double getCurrentPosition(){
    if(! useEncoders){
      Robot.die();
    }

    return elevatorEncoder.getDistance();
  }

  public void setPIDPosition(double distance){
    if(! useEncoders){
      Robot.die();
    }

    setSetpoint(distance);
    targetPosition = distance;
    manualOverride = false;

    // start the timing loop after we set the distance
    // this is needed for PID loop to start working
    getPIDController().enable();  
  }

  public void holdPosition(){
    if(! useEncoders){
      Robot.die();
    }

    setPIDPosition(targetPosition);    
  }

  public void resetEncoder(){
    if(! useEncoders){
      Robot.die();
    }

    elevatorEncoder.reset();
  }

  @Override
  protected double returnPIDInput() { 
    if(! useEncoders){
      Robot.die();
    }

    double pos = getCurrentPosition();

    // stop moving if we are at a limit switch
    // we do this by setting the position we want to
    // move to to the position we are at
    if(atUpperLimit() || atLowerLimit()){
      Robot.Log("PIDElevator: At currpos:" + pos + " lowerlimit:" + atLowerLimit() + " upperlimit:" +atUpperLimit());
      setPIDPosition(pos);
    }

    return pos;  
  }

  @Override
  protected void usePIDOutput(double output) {
    if(! useEncoders){
      Robot.die();
    }

    motor1.pidWrite(output);
  }

  // used to manually set the speed which will disable pid control
  public void setSpeed(double speed){
    currSpeed = speed;
    manualOverride = true;

    // make sure we turn off the pid loop
    getPIDController().disable();
    motor1.set(currSpeed);
  }

  public void Up() {
    if (atUpperLimit()){
      Stop();
    } 
    else {
      VerifyEncoderPhase(pastPosition);
      dirMoved = Robot.Direction.UP;
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
     VerifyEncoderPhase(pastPosition);
     dirMoved = Robot.Direction.DOWN;
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
  private void VerifyEncoderPhase(double prevPos){
    double pos = getCurrentPosition();
    double deltaPos = pos - prevPos;
    double sign = 0;
    boolean check = true;

    // don't do this check when running PID
    // as prev and curr position are not set
    // correctly and could flag a false positive
    // or a false negative
    if(!manualOverride){
      return;
    }

    switch(dirMoved){//direction moved
      case DOWN:
        sign = Math.copySign(1, RobotMap.ElevatorDownSpeed);
        break;
      case UP:
        sign = Math.copySign(1, RobotMap.ElevatorUpSpeed);
        break;
      case NONE:
        check = false;
      break;
    }

    if( check && (Math.abs(deltaPos) > RobotMap.EncoderSlop) ){
      double deltaPosSign = Math.copySign(1, deltaPos);
      if( deltaPosSign != sign){
        Robot.Log("Elevator encoder is out of Phase from Arm Motor dir:" + dirMoved + " deltapos:" + deltaPos);
        Robot.die();
      }
    }
    pastPosition = getCurrentPosition();
    return;
  }

}
