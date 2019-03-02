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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import java.lang.Math;

/**
 * Add your docs here.
 */
public class TalonEncodedWrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final int ENCODER_SLOT_INDEX = 0;
  private static final int PRIMARY_ENCODER_IDX = 0;
  private static final int ENCODER_RESET_POSTION = 0;
  private static final int ENCODER_RESET_TIMEOUT = 0;
  private static final int ENCODER_CONFIG_TIMEOUT = 10;
  private static final int TALONRSX_TIMEOUT = 10;

  private TalonSRX motor1;
  private TalonSRX motor2;

  private double targetPosition;
  private double velocity;
  private boolean isMoving; 
  private boolean encodersAreEnabled = false;
  private boolean limitSwAreEnabled = false;

  //Limit switches;
  private DigitalInput topLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId2);
  private DigitalInput bottomLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId3);

  // holds variables used to determine out of phase encoders
  private Robot.Direction dirMoved = Robot.Direction.NONE; 
  private double pastPosition = 0.0;

  public TalonEncodedWrist() {
    encodersAreEnabled = true;
    limitSwAreEnabled = false;

    isMoving = false;
    targetPosition = 0;
    velocity = 0;

    // assume that motor1 is connected to encoder
    
    motor1 = new TalonSRX(RobotMap.TalonMotorCanID3);
    motor2 = new TalonSRX(RobotMap.TalonMotorCanID4);

    motor1.configFactoryDefault();
    motor2.configFactoryDefault();

    // this could be either true or false, we have to determine
    // how it is confgured
    motor1.setInverted(false);
    motor2.setInverted(false);

     // only 1 controller (motor1) is wired to the encoder, so we have motor2
    // follow motor1 to keep it moving at the same speed
    motor2.follow(motor1);
    
		/* Set the peak and nominal outputs */
		motor1.configNominalOutputForward(0, TALONRSX_TIMEOUT);
		motor1.configNominalOutputReverse(0, TALONRSX_TIMEOUT);
		motor1.configPeakOutputForward(RobotMap.TalonMaxOutput, TALONRSX_TIMEOUT);
    motor1.configPeakOutputReverse(RobotMap.TalonMinOutput, TALONRSX_TIMEOUT);
    
    if (encodersAreEnabled) {
      // init code pulled from https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionMagic/src/main/java/frc/robot/Robot.java

      /* Configure Sensor Source for Pirmary PID */
      motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	
          PRIMARY_ENCODER_IDX, 
          ENCODER_CONFIG_TIMEOUT);

      motor1.setSensorPhase(false);

      /* Set relevant frame periods to be at least as fast as periodic rate */
      /* DJD I don't know what this does                                    */
		  motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ENCODER_CONFIG_TIMEOUT);
		  motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ENCODER_CONFIG_TIMEOUT);

      /* Set Motion Magic gains in slot0 - see documentation */
      motor1.selectProfileSlot(ENCODER_SLOT_INDEX, PRIMARY_ENCODER_IDX);

      //change these parameters
      motor1.config_kF(0, RobotMap.TalonWristPID_F, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kP(0, RobotMap.TalonWristPID_P, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kI(0, RobotMap.TalonWristPID_I, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kD(0, RobotMap.TalonWristPID_D, ENCODER_CONFIG_TIMEOUT);
  
  		/* Set acceleration and vcruise velocity - see documentation */
		  motor1.configMotionCruiseVelocity(RobotMap.TalonWristCruiseSpeed, ENCODER_CONFIG_TIMEOUT);
		  motor1.configMotionAcceleration(RobotMap.TalonWristAcceleration, ENCODER_CONFIG_TIMEOUT);

		  /* Zero the sensor */
      motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_CONFIG_TIMEOUT);
    }

    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);

    Robot.Log("wrist is initialized");
  }

  public void setPIDPosition(double pos) {
    if(!encodersAreEnabled){
      Robot.die();
    }
    Robot.Log("wrist setPidPosition:" + pos);
    targetPosition = pos;
    move();
  }

  public void setTalonSpeed(double val){
    velocity = val;
    Robot.Log("wrist SetTalonSpeed:" + velocity);
    motor1.set(ControlMode.PercentOutput, velocity);  
  }

  public void stop() {
    motor1.neutralOutput();

    // this isn't completely true as this call will start stopping
    // the motor, it may not be completely stopped for an undetermined
    // amount of time
    isMoving = false;
  }

  public boolean resetEncoder(){
    if(!encodersAreEnabled){
      Robot.die();
    }

    stop();
    // make sure motor is stopped when this is called
    // if we are driving the Wrist, the reset point will
    // be indeterminate, we don't want to call this while
    // a move command is running
    if (isMoving) {
      Robot.die();
      return false;
    }
    // there is no guarantee the position will be 0 when this call returns.
    // because it is done asynchronously. Have a command call encoderResetComplete()
    // until it returns true 
    motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_RESET_TIMEOUT);
    Robot.Log("wrist talon encoders reset");
    return true;
  }

  public boolean encoderResetComplete(){
    if(!encodersAreEnabled){
      Robot.die();
    }

    // make sure no move command is running
    if (isMoving) {
      return false;
    }

    boolean complete = false;
    // it may never get to 0, if the motor is moving at all
    // the encoder would return non-zero, we may have to check
    // a range
    if (getCurrentPosition() == 0){
      complete = true;
      Robot.Log("wrist Talon encoder is at 0");
    }
    return complete;
  }

  public double getCurrentPosition() {
    if(!encodersAreEnabled){
      Robot.die();
    }

    double currpos = motor1.getSelectedSensorPosition(0);
    Robot.Log("wrist Talon: currposition(" + currpos + ")");
    return currpos;
  }

  public void holdPosition(){
    if(!encodersAreEnabled){
      Robot.die();
    }
    
    move();
  }

  public void move(){
    if(!encodersAreEnabled){
      Robot.die();
    }
    Robot.Log("wrist moving to target position:" + targetPosition);
    //isMoving = true;
    motor1.set(ControlMode.MotionMagic, targetPosition);
  }

  public boolean onTarget(){
    if(!encodersAreEnabled){
      Robot.die();
    }
    
    boolean reachedTarget = false;
    boolean belowRange = true;
    boolean aboveRange = true;
    double curpos = getCurrentPosition();

    belowRange = curpos < (targetPosition - RobotMap.TalonWristAbsTolerance);
    aboveRange = curpos > (targetPosition + RobotMap.TalonWristAbsTolerance);

    reachedTarget = ((!belowRange) && (!aboveRange));
    Robot.Log("wrist OnTarget:" + reachedTarget + " (" + belowRange + "," + aboveRange + ")");
    return reachedTarget;
  }

  public void logEncoderValues(){
    Robot.Log("wrist Talon current pos:" + getCurrentPosition());
  }

  public void Up(){
    //Robot.Log("TalonEncoded: reached up");
    if (atUpperLimit()){
      stop();
    }
    else {
      VerifyEncoderPhase(pastPosition);
      dirMoved = Robot.Direction.UP;
      pastPosition = getCurrentPosition();
      setTalonSpeed(RobotMap.TalonWristUpSpeed);
      logEncoderValues();
    }
  }

  public void Down(){
    if (atLowerLimit()){
      stop();
    }
    else {
      VerifyEncoderPhase(pastPosition);
      dirMoved = Robot.Direction.DOWN;
      pastPosition = getCurrentPosition();
      setTalonSpeed(RobotMap.TalonWristDownSpeed);
      logEncoderValues();
    }
  }

  public void Stop(){
    // or call motor1.stopMotor();
    setTalonSpeed(RobotMap.TalonWristStopSpeed);
    stop();
    logEncoderValues();
  }

   public boolean atUpperLimit(){
    //return topLimit.get();
    return false;
  }

  public boolean atLowerLimit() {
   // return bottomLimit.get();
      return false;

  }

  public void dumpLimitSwitchValues(){
    Robot.Log("wrist talon: atLowerLimit:" + atLowerLimit() + " atUpperLimit:" + atUpperLimit());
  }  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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
    
    if( check && (deltaPos != 0) ){
      double deltaPosSign = Math.copySign(1, deltaPos);
      if( deltaPosSign != sign){
        inPhase = false;
        Robot.Log("Wrist encoder is out of Phase from Wrist Motor");
        Robot.die();
      }
    }
    return inPhase;
  }
}
