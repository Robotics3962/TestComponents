/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.String;

/**
 * Add your docs here.
 */
public class FrcError {
    public static enum Error{
        Ok,
        InvalidOperation
    }

    Error errorCode = Error.Ok;
    String errorInfo = "";

    public FrcError(){
    }

    public FrcError(Error errCode){
        errorCode = errCode;
    }

    public FrcError(Error errCode, String info){
        errorCode = errCode;
        errorInfo = info;
    }

    public void set(Error errCode){
        errorCode = errCode;
        errorInfo = "";
    }

    public void set(Error errCode, String info){
        errorCode = errCode;
        errorInfo = info;
    }

    public boolean isOk(){
        return errorCode == Error.Ok;
    }

    public Error getError(){
        return errorCode;
    }

    public String getText(){
        return "" + errorCode + ": " + errorInfo; 
    }
}
