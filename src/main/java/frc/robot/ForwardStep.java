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
 * A specific autonomous command used for moving the robot a specific number of inches.
 */
public class ForwardStep implements AutonomousStep{

    //Constants specific to the robot's dimensions used in later calculations.
    //Diameter of the wheels, in inches.
    public static final double WHEEL_DIAMETER = 6;
    //Gear Ratio between the motor and the wheels
    public static final double GEAR_RATIO = 12.75;

    //Distance the robot must travel to complete the motion of the robot. This is the goal of the command that must be met before the next command can execute.
    private double distance;

    //Error, in inches, of the distance that is allowable before the command can finish. This is used, as an exact distance can never be reached. Reduce this value to get more accurate positioning.
    private double error = 2*GEAR_RATIO;

    /**
     * Creates a translation command with a specified distance of travel in inches. Positive values yield counter-clockwise rotation.
     */
    public ForwardStep(double distance)
    {
        this.distance = distance*GEAR_RATIO;
    }

    /**
     * Function called when the motion of the robot must be updated, based on the desired position goal specified.
     * Position based control is used, and distance is converted into number of rotations required to meet this position.
     */
    public void execute(CANPIDController leftPidController, CANPIDController rightPidController) 
    {
        leftPidController.setReference(distance/(Math.PI*WHEEL_DIAMETER), ControlType.kPosition);
        rightPidController.setReference(-distance/(Math.PI*WHEEL_DIAMETER), ControlType.kPosition);
    }

    /**
     * Function called when checking to see if the robot has moved the desirable amount within the specified allowable error.
     */
    public boolean checkGoal(CANEncoder leftEncoder, CANEncoder rightEncoder)
    {
        double leftPos = leftEncoder.getPosition()*Math.PI*WHEEL_DIAMETER;
        double rightPos = -rightEncoder.getPosition()*Math.PI*WHEEL_DIAMETER;
        return leftPos < distance+error && leftPos > distance-error && rightPos < distance+error && rightPos > distance-error;
    }
}
