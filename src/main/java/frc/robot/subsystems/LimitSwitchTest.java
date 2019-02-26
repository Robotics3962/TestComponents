/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class LimitSwitchTest extends Subsystem {

private DigitalInput limitSwitch = null; 

public LimitSwitchTest(){
  limitSwitch = new DigitalInput(RobotMap.LimitSwitchPIOId1);
}

public void LogValues()
{
  System.out.println("Limitswitch value is (" + limitSwitch.get() + ")");
}

@Override
  public void initDefaultCommand() {
  }
}
