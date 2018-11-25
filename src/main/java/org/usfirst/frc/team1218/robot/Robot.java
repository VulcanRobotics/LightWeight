/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1218.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;


import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot{
	AHRS ahrs;
	
/*  Initialize PID controller for Turn to degrees
	PIDController turnController;
	double rotateToAngleRate;
	// The following PID Controller coefficients will need to be tuned 
    // to match the dynamics of your drive system.  Note that the      
    // SmartDashboard in Test mode has support for helping you tune    
    // controllers by displaying a form where you can enter new P, I,  
    // and D constants and test the mechanism.                         
    
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;
*/

	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private WPI_TalonSRX LeftDrive1;
	private WPI_TalonSRX LeftDrive2;
	private WPI_TalonSRX LeftDrive3;
	private WPI_TalonSRX LeftDrive4;
	private WPI_TalonSRX RightDrive1;
	private WPI_TalonSRX RightDrive2;
	private WPI_TalonSRX RightDrive3;
	private WPI_TalonSRX RightDrive4;
	private Joystick DriverJoystick;
	private DifferentialDrive DriveBase;	
	private int MaxAmps = 40;
	private int MaxAmpsTimeOut = 1000;
	//private boolean isInverted = true;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("Hello from Gradle!!");
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		// Define Left drive with Even CAN IDs
		LeftDrive1 = new WPI_TalonSRX(52);
		LeftDrive2 = new WPI_TalonSRX(54);
		LeftDrive3 = new WPI_TalonSRX(56);
		LeftDrive4 = new WPI_TalonSRX(58);
		
		//Invert motor direction
		//LeftDrive1.setInverted(isInverted);
		//LeftDrive2.setInverted(isInverted);
		//LeftDrive3.setInverted(isInverted);
		//LeftDrive4.setInverted(isInverted);
		
		// Define right drive with Odd CAN IDs
		RightDrive1 = new WPI_TalonSRX(51);
		RightDrive2 = new WPI_TalonSRX(53);
		RightDrive3 = new WPI_TalonSRX(55);
		RightDrive4 = new WPI_TalonSRX(57);
		
		//Invert motor direction
		//RightDrive1.setInverted(isInverted);
		//RightDrive2.setInverted(isInverted);
		//RightDrive3.setInverted(isInverted);
		//RightDrive4.setInverted(isInverted);
		
		DriverJoystick = new Joystick(0);
		
		// Set up 3 motor controllers to follow master
		LeftDrive2.set(ControlMode.Follower, LeftDrive1.getDeviceID());
		LeftDrive3.set(ControlMode.Follower, LeftDrive1.getDeviceID());
		LeftDrive4.set(ControlMode.Follower, LeftDrive1.getDeviceID());
		
		RightDrive2.set(ControlMode.Follower, RightDrive1.getDeviceID());
		RightDrive3.set(ControlMode.Follower, RightDrive1.getDeviceID());
		RightDrive4.set(ControlMode.Follower, RightDrive1.getDeviceID());
		
		// Link motors into arcade or Differntial Drive
		DriveBase = new DifferentialDrive(LeftDrive1, RightDrive1);
		//DriveBase = new CurvatureDrive(LeftDrive1, RightDrive1);
	
		// Set current limits on motors on Master, Slaves should follow
		LeftDrive1.configContinuousCurrentLimit(MaxAmps,MaxAmpsTimeOut);
		RightDrive1.configContinuousCurrentLimit(MaxAmps,MaxAmpsTimeOut);
		
		
		//set up camera server
		CameraServer.getInstance().startAutomaticCapture();
		
		  try {
				/***********************************************************************
				 * navX-MXP:
				 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
				 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
				 * 
				 * navX-Micro:
				 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
				 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
				 * 
				 * Multiple navX-model devices on a single robot are supported.
				 ************************************************************************/
	            ahrs = new AHRS(SerialPort.Port.kUSB1);
	            //ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
	            ahrs.enableLogging(true);
	        } catch (RuntimeException ex ) {
	            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
	        }
	        Timer.delay(1.0);
	        
	        
	        LeftDrive1.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	        RightDrive1.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	        
	        /* Setup sensors to check status, can also be used for phasing */
	        RightDrive1.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);
	        RightDrive1.setSensorPhase(false);
	        LeftDrive1.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);
	        LeftDrive1.setSensorPhase(false);
   
	        
	        
	        
	        
	        
	}

	
	
	
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	
	// Configure Arcade drive note Joystick Y direction is reversed
	DriveBase.curvatureDrive(-DriverJoystick.getY(),DriverJoystick.getX(),DriverJoystick.getRawButton(1));
	SmartDashboard.putNumber("Velocity: ", -DriverJoystick.getY());
	SmartDashboard.putNumber("Turning:  ", DriverJoystick.getX());
	SmartDashboard.putBoolean("Quick:  ", DriverJoystick.getTrigger());
	
	// Note you must enable teleop to populate new widgets
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

    // use thumb button to reset gyro yaw
    if ( DriverJoystick.getTop()) 
    	{
        	ahrs.zeroYaw();
    	}
    

    /* Output value to SmartDashboard */
    SmartDashboard.putNumber("Right Sensor position", RightDrive1.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Left Sensor Velocity", LeftDrive1.getSelectedSensorVelocity(0));
 
    
	}

	

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		  		
	}
}
