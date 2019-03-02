/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Robot.Direction;
import frc.robot.Robot;
import frc.robot.commands.ArmHoldCmd;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; 
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import java.lang.Math;

public class TalonEncodedArm extends Subsystem {

  private static final int ENCODER_SLOT_INDEX = 0;
  private static final int PRIMARY_ENCODER_IDX = 0;
  private static final int ENCODER_RESET_POSTION = 0;
  private static final int ENCODER_RESET_TIMEOUT = 0;
  private static final int ENCODER_CONFIG_TIMEOUT = 10;
  private static final int TALONRSX_TIMEOUT = 10;

  private WPI_TalonSRX motor1;
  private WPI_TalonSRX motor2;
  //private TalonSRX motor1;
  //private TalonSRX motor2;
  private double targetPosition;
  private double velocity;
  private boolean encodersAreEnabled = false;
  private boolean limitSwAreEnabled = false;
  private boolean manualOverride = true;
  private int count = 0;
  private int logMsgInterval = 5;

  // set to false to use position, set to true to
  // use motion magic.  girls of steel uses motion magic for
  // what looks to be an elevator, but not for what looks
  // like a rotation
  private boolean useMotionMagic = false;

  // holds variables used to determine out of phase encoders
  private Robot.Direction dirMoved = Robot.Direction.NONE; 
  private double pastPosition = 0.0;

  //Limit switches;
  private DigitalInput topLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId2);
  private DigitalInput bottomLimit = null; //new DigitalInput(RobotMap.LimitSwitchPIOId3);

  public TalonEncodedArm() {
    encodersAreEnabled = true;
    limitSwAreEnabled = false;

    targetPosition = 0;
    velocity = 0;

    // assume that motor1 is connected to encoder
    motor1 = new WPI_TalonSRX(RobotMap.TalonMotorCanId1);
    motor2= new WPI_TalonSRX(RobotMap.TalonMotorCanId2); 
    //motor1 = new TalonSRX(RobotMap.TalonMotorCanId1);
    //motor2= new TalonSRX(RobotMap.TalonMotorCanId2); 
   
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();

    // only 1 controller (motor1) is wired to the encoder, so we have motor2
    // follow motor1 to keep it moving at the same speed
    motor2.follow(motor1);
    
    /* Set the peak and nominal outputs */
    /* gos doesn't call these functions */
		motor1.configNominalOutputForward(0, TALONRSX_TIMEOUT);
		motor1.configNominalOutputReverse(0, TALONRSX_TIMEOUT);
		motor1.configPeakOutputForward(RobotMap.TalonMaxOutput, TALONRSX_TIMEOUT);
    motor1.configPeakOutputReverse(RobotMap.TalonMinOutput, TALONRSX_TIMEOUT);

    // this could be either true or false, we have to determine
    // how it is confgured
    motor1.setInverted(false);
    motor2.setInverted(false);

    if (encodersAreEnabled) {
      // init code pulled from https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionMagic/src/main/java/frc/robot/Robot.java

      /* Configure Sensor Source for Pirmary PID */
      /* GOS does not call this function, I guess they take the default */
      /* they could be using the absolute location mode */
      motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	
          PRIMARY_ENCODER_IDX, 
          ENCODER_CONFIG_TIMEOUT);

      motor1.setSensorPhase(true);

      /* Set relevant frame periods to be at least as fast as periodic rate */
      /* DJD I don't know what this does                                    */
      /* girls of steel doesn't do this                                     */
		  //motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ENCODER_CONFIG_TIMEOUT);
		  //motor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ENCODER_CONFIG_TIMEOUT);

      /* Set Motion Magic gains in slot0 - see documentation */
      /* girls of steel uses motion magic for elevator, but position */
      /* for the pivot, so copy them                         */
      motor1.selectProfileSlot(ENCODER_SLOT_INDEX, PRIMARY_ENCODER_IDX);

      motor1.config_kF(0, RobotMap.TalonArmPID_F, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kP(0, RobotMap.TalonArmPID_P, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kI(0, RobotMap.TalonArmPID_I, ENCODER_CONFIG_TIMEOUT);
      motor1.config_kD(0, RobotMap.TalonArmPID_D, ENCODER_CONFIG_TIMEOUT);
  
      if(useMotionMagic){
  		  /* Set acceleration and vcruise velocity - see documentation */
		    motor1.configMotionCruiseVelocity(RobotMap.TalonArmCruiseSpeed, ENCODER_CONFIG_TIMEOUT);
        motor1.configMotionAcceleration(RobotMap.TalonArmAcceleration, ENCODER_CONFIG_TIMEOUT);
      }

		  /* Zero the sensor */
      motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_CONFIG_TIMEOUT);
    }

    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);

    Robot.Log("Talon is initialized");
  }

  public void setPIDPosition(double pos) {
    if(!encodersAreEnabled){
      Robot.die();
    }

    // this is called for PID control so turn off
    // manual override, also turn off phase
    // checking
    manualOverride = false;
    dirMoved = Direction.NONE;

    Robot.Log("Arm setPidPosition:" + pos);
    targetPosition = pos;

    // don't need to move the arm as the background command
    // will take care of that
  }

  public void stop() {
    //motor1.neutralOutput();
    motor1.stopMotor();
  }

  public boolean resetEncoder(){
    if(!encodersAreEnabled){
      Robot.die();
    }

    // stop the motor as we shouldn't reset when moving
    stop();

    // there is no guarantee the position will be 0 when this call returns.
    // because it is done asynchronously. Have a command call encoderResetComplete()
    // until it returns true 
    motor1.setSelectedSensorPosition(PRIMARY_ENCODER_IDX, ENCODER_RESET_POSTION, ENCODER_RESET_TIMEOUT);
    Robot.Log("talon encoders reset");
    return true;
  }

  public boolean encoderResetComplete(){
    if(!encodersAreEnabled){
      Robot.die();
    }

    boolean complete = false;
    // it may never get to 0, if the motor is moving at all
    // the encoder would return non-zero, we may have to check
    // a range
    if (getCurrentPosition() == 0){
      complete = true;
      Robot.Log("Arm Talon encoder reset is complete");
    }
    return complete;
  }

  public double getTargetPosition(){
    return targetPosition;
  }
  
  public double getCurrentPosition() {
    if(!encodersAreEnabled){
      Robot.die();
    }

    double currpos = motor1.getSelectedSensorPosition(0);
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

    // only move via PID when we are not manually controlling
    if(!manualOverride){
      if(useMotionMagic){
        motor1.set(ControlMode.MotionMagic, targetPosition);
      }
      else {
        motor1.set(ControlMode.Position, targetPosition);
      }
    }

    LogInfo(true);
  }

  public boolean onTarget(){
    if(!encodersAreEnabled){
      Robot.die();
    }
    
    boolean reachedTarget = false;
    boolean belowRange = true;
    boolean aboveRange = true;
    double curpos = getCurrentPosition();

    belowRange = curpos < (targetPosition - RobotMap.TalonArmAbsTolerance);
    aboveRange = curpos > (targetPosition + RobotMap.TalonArmAbsTolerance);

    reachedTarget = ((!belowRange) && (!aboveRange));
    Robot.Log("OnTarget:" + reachedTarget + " (" + belowRange + "," + aboveRange + ")");
    return reachedTarget;
  }

  public void initDefaultCommand(){
    setDefaultCommand(new ArmHoldCmd());
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
        sign = Math.copySign(1, RobotMap.TalonArmDownSpeed);
        break;
      case UP:
        sign = Math.copySign(1, RobotMap.TalonArmUpSpeed);
        break;
      case NONE:
        check = false;
      break;
    }

    if( check && (Math.abs(deltaPos) > RobotMap.EncoderSlop) ){
      double deltaPosSign = Math.copySign(1, deltaPos);
      if( deltaPosSign != sign){
        Robot.Log("Arm encoder is out of Phase from Arm Motor");
        Robot.die();
      }
    }
    pastPosition = getCurrentPosition();
    return;
  }

  // these commands are used to manually move the arm
  // outside of the PID control

  public void setTalonSpeed(double val){
    // this command is used to manually move the
    // motor so set the variable that stops the PID
    // from overriding 
    manualOverride = true;
    velocity = val;
    motor1.set(ControlMode.PercentOutput, velocity);
    LogInfo(true);  
  }

  public void Up(){
    if (atUpperLimit()){
      stop();
    }
    else {
      VerifyEncoderPhase(pastPosition);
      // do not set this before verify
      dirMoved = Robot.Direction.UP;
      setTalonSpeed(RobotMap.TalonArmUpSpeed);
    }
  }

  public void Down(){
    if (atLowerLimit()){
      stop();
    }
    else {
      VerifyEncoderPhase(pastPosition);
      // do not set this before verify
      dirMoved = Robot.Direction.DOWN;
      setTalonSpeed(RobotMap.TalonArmDownSpeed);
    }
  }

  public void Stop(){
    dirMoved = Direction.NONE;
    // or call motor1.stopMotor()
    stop();
  }
  public void LogInfo(boolean dampen){
    count++;

    if(dampen && ((count % logMsgInterval) != 0)){
      return;
    }
    
    double currPos = getCurrentPosition();

    String output = "Arm   Info: manual:" + manualOverride;
    output = output + " target:" + targetPosition;
    output = output + " current:" + currPos;
    output = output + " ontarg:" + onTarget();
    output = output + " dir:" + dirMoved;
    output = output + " speed:" + velocity;
    Robot.Log(output);
  }
}

