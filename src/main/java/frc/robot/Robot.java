/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//Import WPI libraries for general FRC functionality
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


//Import libraries for the motor controllers
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

  //Prepare room in memory for the following variables, which will be assigned later.

  //Drive system used for tank drive motion of the robot during tele-op (Replace with MecanumDrive if using mecanum wheels)
  private DifferentialDrive m_myRobot;
  //Game controller used for controlling the robot during tele-op
  private XboxController m_controller;

  //Can Bus ID's of the right and left motor controllers, respectively
  private static final int rightDeviceID = 1; 
  private static final int leftDeviceID = 2;
  //Left and right motor controller and motor pairs.
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  //left and right PID controllers, internal to each motor controller, respectively.
  private CANPIDController m_leftPidController, m_rightPidController;
  //left and right encoders, internal to each motor controller, respectively.
  private CANEncoder m_leftEncoder, m_rightEncoder;
  //Constants used to tune the PID controllers of both motor controllers.
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    //Get the first (0 is first in java) xbox controller plugged into the computer.
    m_controller = new XboxController(0);

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_leftPidController = m_leftMotor.getPIDController();
    m_rightPidController = m_rightMotor.getPIDController();

    /**
     * In order to use encoder functionality for a controller, a CANEncoder object
     * is constructed by calling the getEncoder() method on an existing
     * CANSparkMax object
     */
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();

    /**
     * PID coefficients and the maximum and minimum speed allowed for the motor controllers to run at.
     * 1/4 speed used for safer testing.
     * It is highly recommended to tune PID variables to get more predictable or desirable robot behavior.
     */ 
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.25; 
    kMinOutput = -0.25;

    // Set the PID coefficients for both the left and right motor controllers
    m_leftPidController.setP(kP);
    m_leftPidController.setI(kI);
    m_leftPidController.setD(kD);
    m_leftPidController.setIZone(kIz);
    m_leftPidController.setFF(kFF);
    m_leftPidController.setOutputRange(kMinOutput, kMaxOutput);

    m_rightPidController.setP(kP);
    m_rightPidController.setI(kI);
    m_rightPidController.setD(kD);
    m_rightPidController.setIZone(kIz);
    m_rightPidController.setFF(kFF);
    m_rightPidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    //Create the differential drive using the left and right drive motors.
    //Will be used for passing in commands for tank drive motion later.
    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
  }

  @Override
  public void teleopPeriodic() {
    //Gets the forward and backwards position of the left and right joysticks on the xbox controller.
    //These positions are then scaled to robot velocity using the scaleSpeed() function
    double leftSpeed = scaleSpeed(m_controller.getY(Hand.kLeft));
    double rightSpeed = scaleSpeed(m_controller.getY(Hand.kRight));
    //Scaled speeds are passed into the tankDrive function to control the speed of the robot's differential drive.
    m_myRobot.tankDrive(leftSpeed,rightSpeed);
  }

  //A list of autonomous commands is created. Commands will be executed from first to last. See each commands constructor/class for more specific details.
  public AutonomousStep[] steps = {new ForwardStep(60), new RotateStep(-90), new ForwardStep(36)};
  //Start at the first command (0 is first in java)
  public int currentStep = 0;

  @Override
  public void autonomousInit() {
    super.autonomousInit();
    //Reset the encoders so any commands are relative to the robot's initial position.
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  @Override
  public void autonomousPeriodic() {
    //Update the robot's current position/rotation based on the current command's goal.
    steps[currentStep].execute(m_leftPidController, m_rightPidController); 

    //Check to see if the current command has met it's goal.
    //If it has, the next command is started, the encoders are reset, and the motors are stopped to prevent coasting.
    if(steps[currentStep].checkGoal(m_leftEncoder, m_rightEncoder) == true)
    {
      currentStep++;
      m_leftEncoder.setPosition(0);
      m_rightEncoder.setPosition(0);
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    }
  }

  /**
   * Function used to scale the input "speed" (usually the position of a joystick) to the output speed of a motor.
   * This is used to achieve more predictable/desirable motion of the robot.
   * The speed is squared to reduce low speed sensitivity, and halved to prevent high speeds for testing.
   * Direction is preserved.
   * 
   * @param inputSpeed
   * @return motorSpeed
   */
  public double scaleSpeed(double inputSpeed)
  {
    return -inputSpeed*inputSpeed*Math.signum(inputSpeed)*0.5;
  }
}
