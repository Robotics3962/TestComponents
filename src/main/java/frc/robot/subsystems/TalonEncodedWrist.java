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

  private TalonSRX motor3;
  private TalonSRX motor4;
  private double targetPosition;
  private double velocity;
  private boolean isMoving; 
  private boolean encodersAreEnabled = false;
  private boolean limitSwAreEnabled = false;

  //Limit switches;
  private DigitalInput topLimit = new DigitalInput(RobotMap.LimitSwitchPIOId2);
  private DigitalInput bottomLimit = new DigitalInput(RobotMap.LimitSwitchPIOId3);

  // this is a divide by 0 which will 
  // throw an exception which should 
  // stop the program from running or otherwise
  // indicate an erro
  private void die(){
    int x = 0;
    int u = 1/x;
  }

  public TalonEncodedWrist() {
    encodersAreEnabled = true;
    limitSwAreEnabled = false;

    isMoving = false;
    targetPosition = 0;
    velocity = 0;

    // assume that motor1 is connected to encoder
    
    motor3 = new TalonSRX(RobotMap.TalonMotorCanID3);
    motor4 = new TalonSRX(RobotMap.TalonMotorCanID4);

    motor3.configFactoryDefault();
    motor4.configFactoryDefault();

     // only 1 controller (motor1) is wired to the encoder, so we have motor2
    // follow motor1 to keep it moving at the same speed
    motor4.follow(motor3);
    
		/* Set the peak and nominal outputs */
		motor3.configNominalOutputForward(0, TALONRSX_TIMEOUT);
		motor4.configNominalOutputReverse(0, TALONRSX_TIMEOUT);
		motor3.configPeakOutputForward(RobotMap.TalonMaxOutput, TALONRSX_TIMEOUT);
    motor3.configPeakOutputReverse(RobotMap.TalonMinOutput, TALONRSX_TIMEOUT);

    // this could be either true or false, we have to determine
    // how it is confgured
    motor3.setInverted(false);
    motor4.setInverted(false);

    if (encodersAreEnabled) {
      // init code pulled from https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionMagic/src/main/java/frc/robot/Robot.java

      /* Configure Sensor Source for Pirmary PID */
      motor3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	
          PRIMARY_ENCODER_IDX, 
          ENCODER_CONFIG_TIMEOUT);

      motor3.setSensorPhase(true);

      /* Set relevant frame periods to be at least as fast as periodic rate */
      /* DJD I don't know what this does                                    */
		  motor3.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ENCODER_CONFIG_TIMEOUT);
		  motor3.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ENCODER_CONFIG_TIMEOUT);

      /* Set Motion Magic gains in slot0 - see documentation */
      motor3.selectProfileSlot(ENCODER_SLOT_INDEX, PRIMARY_ENCODER_IDX);

      //change these parameters
      motor3.config_kF(0, RobotMap.TalonPID_F, ENCODER_CONFIG_TIMEOUT);
      motor3.config_kP(0, RobotMap.TalonPID_P, ENCODER_CONFIG_TIMEOUT);
      motor3.config_kI(0, RobotMap.TalonPID_I, ENCODER_CONFIG_TIMEOUT);
      motor3.config_kD(0, RobotMap.TalonPID_D, ENCODER_CONFIG_TIMEOUT);
  
  		/* Set acceleration and vcruise velocity - see documentation */
		  motor3.configMotionCruiseVelocity(RobotMap.TalonCruiseSpeed, ENCODER_CONFIG_TIMEOUT);
		  motor3.configMotionAcceleration(RobotMap.TalonAcceleration, ENCODER_CONFIG_TIMEOUT);

		  /* Zero the sensor */
      motor3.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_CONFIG_TIMEOUT);
    }

    motor3.setNeutralMode(NeutralMode.Brake);
    motor4.setNeutralMode(NeutralMode.Brake);

    Robot.Log("Talon is initialized");
  }

  public void setPIDPosition(double pos) {
    if(!encodersAreEnabled){
      die();
    }
    Robot.Log("setPidPosition:" + pos);
    targetPosition = pos;
    move();
  }

  public void setTalonSpeed(double val){
    velocity = val;
    Robot.Log("SetTalonSpeed:" + velocity);
    motor3.set(ControlMode.PercentOutput, velocity);  
  }

  public void stop() {
    motor3.neutralOutput();

    // this isn't completely true as this call will start stopping
    // the motor, it may not be completely stopped for an undetermined
    // amount of time
    isMoving = false;
  }

  public boolean resetEncoder(){
    if(!encodersAreEnabled){
      die();
    }

    stop();
    // make sure motor is stopped when this is called
    // if we are driving the Wrist, the reset point will
    // be indeterminate, we don't want to call this while
    // a move command is running
    if (isMoving) {
      die();
      return false;
    }
    // there is no guarantee the position will be 0 when this call returns.
    // because it is done asynchronously. Have a command call encoderResetComplete()
    // until it returns true 
    motor3.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_RESET_TIMEOUT);
    Robot.Log("talon encoders reset");
    return true;
  }

  public boolean encoderResetComplete(){
    if(!encodersAreEnabled){
      die();
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
      Robot.Log("Talon encoder is at 0");
    }
    return complete;
  }

  public double getCurrentPosition() {
    if(!encodersAreEnabled){
      die();
    }

    double currpos = motor3.getSelectedSensorPosition(0);
    Robot.Log("Talon: currposition(" + currpos + ")");
    return currpos;
  }

  public void holdPosition(){
    if(!encodersAreEnabled){
      die();
    }
    
    move();
  }

  public void move(){
    if(!encodersAreEnabled){
      die();
    }
    Robot.Log("moving to target position:" + targetPosition);
    //isMoving = true;
    motor3.set(ControlMode.MotionMagic, targetPosition);
  }

  public boolean onTarget(){
    if(!encodersAreEnabled){
      die();
    }
    
    boolean reachedTarget = false;
    boolean belowRange = true;
    boolean aboveRange = true;
    double curpos = getCurrentPosition();

    belowRange = curpos < (targetPosition - RobotMap.TalonAbsTolerance);
    aboveRange = curpos > (targetPosition + RobotMap.TalonAbsTolerance);

    reachedTarget = ((!belowRange) && (!aboveRange));
    Robot.Log("OnTarget:" + reachedTarget + " (" + belowRange + "," + aboveRange + ")");
    return reachedTarget;
  }

  public void logEncoderValues(){
    Robot.Log("Talon current pos:" + getCurrentPosition());
  }

  public void Up(){
    //Robot.Log("TalonEncoded: reached up");
    if (atUpperLimit()){
      stop();
    }
    else {
      setTalonSpeed(RobotMap.TalonUpSpeed);
      logEncoderValues();
    }
  }

  public void Down(){
    if (atLowerLimit()){
      stop();
    }
    else {
      setTalonSpeed(RobotMap.TalonDownSpeed);
      logEncoderValues();
    }
  }

  public void Stop(){
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
    Robot.Log("talon: atLowerLimit:" + atLowerLimit() + " atUpperLimit:" + atUpperLimit());
  }  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

 
}
