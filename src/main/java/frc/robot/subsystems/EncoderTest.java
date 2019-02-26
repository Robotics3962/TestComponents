/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class EncoderTest extends Subsystem {
  Encoder encoder = null;
  Encoder encoder2 = null;
  Encoder encoder3 = null;

  public EncoderTest(){
    encoder = new Encoder(RobotMap.EncoderPIOId1, RobotMap.EncoderPIOId2,false, Encoder.EncodingType.k4X);

   // encoder2 = new Encoder(RobotMap.EncoderPIOId3, RobotMap.EncoderPIOId4, false, Encoder.EncodingType.k4X);

    //encoder3 = new Encoder(RobotMap.EncoderPIOId5, RobotMap.EncoderPIOId6, false, Encoder.EncodingType.k4X);

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
    System.out.println("encoder: get:" + val + " stopped:" + valS + " raw:" + valR +  " dist:" + valD);

    /*
    int val2 = encoder.get();
    boolean valS2 = encoder.getStopped();
    double valR2 = encoder.getRaw();
    double valD2 = encoder.getDistance();
    System.out.println("encoder2: get:" + val2 + " stopped:" + valS2 + " raw:" + valR2 +  " dist:" + valD2);

    int val3 = encoder.get();
    boolean valS3 = encoder.getStopped();
    double valR3 = encoder.getRaw();
    double valD3 = encoder.getDistance();
    System.out.println("encoder3: get:" + val3 + " stopped:" + valS3 + " raw:" + valR3 +  " dist:" + valD3);
*/




  }
}
