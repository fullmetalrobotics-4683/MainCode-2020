/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//Import libraries for the motor controllers
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

/**
 * A specific autonomous command used for rotating the robot a specific number of degrees.
 */
public class RotateStep implements AutonomousStep{

    //Constants specific to the robot's dimensions used in later calculations.
    //Diameter of the wheels, in inches.
    public static final double WHEEL_DIAMETER = 6;
    //Width of the robot in inches (measured between the middle of the wheels.)
    public static final double ROBOT_WIDTH = 23.5;
    //Gear Ratio between the motor and the wheels
    public static final double GEAR_RATIO = 12.75;

    //Length of travel each side of the robot must travel to complete the rotation of the robot. This is the goal of the command that must be met before the next command can execute.
    private double arcLength;

    //Error, in inches, of the arc length that is allowable before the command can finish. This is used, as an exact distance can never be reached. Reduce this value to get more accurate turns.
    private double error = 0.25;


    /**
     * Creates a rotation command with a specified angle of rotation in degrees. Positive values yield counter-clockwise rotation.
     */
    public RotateStep(double angle)
    {
        //Calculate the desired arc length each side must travel, based on the desired degrees of rotation and dimensions of the robot.
        this.arcLength = Math.toRadians(angle)*ROBOT_WIDTH/2.0;
    }

    /**
     * Function called when the motion of the robot must be updated, based on the desired rotation goal specified.
     * Position based control is used, and arc legth is converted into number of rotations required to meet this arc length.
     */
    public void execute(CANPIDController leftPidController, CANPIDController rightPidController) 
    {
        leftPidController.setReference(-GEAR_RATIO*arcLength/(Math.PI*WHEEL_DIAMETER), ControlType.kPosition);
        rightPidController.setReference(-GEAR_RATIO*arcLength/(Math.PI*WHEEL_DIAMETER), ControlType.kPosition);
    }

    /**
     * Function called when checking to see if the robot has rotated the desirable amount within the specified allowable error.
     */
    public boolean checkGoal(CANEncoder leftEncoder, CANEncoder rightEncoder)
    {
        double leftPos = -leftEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_RATIO;
        double rightPos = -rightEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_RATIO;

        return leftPos < arcLength+error && leftPos > arcLength-error && rightPos < arcLength+error && rightPos > arcLength-error;
    }
}
