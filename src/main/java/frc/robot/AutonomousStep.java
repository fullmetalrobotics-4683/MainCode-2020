/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
/**
 * A Base Class that specifies that each autonomous command/step must have a way to execute and check if it is done.
 */
public interface AutonomousStep {


    //This specifies that every specific autonomous command must have a way to execute/move the robot, given a left and right PID controller.
    public void execute(CANPIDController leftPidController, CANPIDController rightPidController);

    //This specifies that every specific autonomous command must have a way to check if the command is finished running, given a left and right encoder.
    public boolean checkGoal(CANEncoder leftEncoder, CANEncoder rightEncoder);
}
