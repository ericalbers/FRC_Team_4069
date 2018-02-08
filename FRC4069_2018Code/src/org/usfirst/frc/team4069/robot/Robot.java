/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.SampleRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * <p>The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 *
 * <p>WARNING: While it may look like a good choice to use for your code if
 * you're inexperienced, don't. Unless you know what you are doing, complex code
 * will be much more difficult under this system. Use IterativeRobot or
 * Command-Based instead if you're new.
 * vacuum is 21
 * 
 */
public class Robot extends SampleRobot 
{
	WPI_TalonSRX leftDriveTalon11Follower = new WPI_TalonSRX(IOMapping.LEFT_DRIVE_CAN_BUS_11_FOLLOWER);
	WPI_TalonSRX leftDriveTalon12Driver = new WPI_TalonSRX(IOMapping.LEFT_DRIVE_CAN_BUS_12_DRIVER);
	WPI_TalonSRX leftDriveTalon13Follower = new WPI_TalonSRX(IOMapping.LEFT_DRIVE_CAN_BUS_13_FOLLOWER);
	
	WPI_TalonSRX rightDriveTalon18Follower = new WPI_TalonSRX(IOMapping.RIGHT_DRIVE_CAN_BUS_FOLLOWER);
	WPI_TalonSRX rightDriveTalon19Driver = new WPI_TalonSRX(IOMapping.RIGHT_DRIVE_CAN_BUS19_DRIVER);
	WPI_TalonSRX rightDriveTalon20Follower = new WPI_TalonSRX(IOMapping.RIGHT_DRIVE_CAN_BUS20_FOLLOWER);
	
	TalonSRX mElevatorTalon = new TalonSRX(16); //elevator
	TalonSRX mVacuumTalon = new TalonSRX(21); //vacuum
	
	ControlElevator mControlElevator = new ControlElevator(this);
	ControlWinch mControlWinch = new ControlWinch();
	
	
	public LowPassFilter leftDriveMotorLowPassFilter;
	public LowPassFilter rightDriveMotorLowPassFilter;

	double mDriverRobotSpeed=0.0;
	double mDriverRobotTurnDirection = 0.0;
	
	
	//Joystick _joy = new Joystick(0);
	Joystick driverStick = new Joystick(0); // set to ID 1 in DriverStation
	Joystick controlStick = new Joystick(1); // set to ID 2 in DriverStation

	
	public boolean ON_RED_SIDE_OF_FIELD = false;
	
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";

	
	public double mElevatorSpeed = 0.0;
	
	
	long mLastDashboardUpdateTime = 0;

	
	private DifferentialDrive m_robotDrive = new DifferentialDrive(leftDriveTalon11Follower, rightDriveTalon18Follower);
	private Joystick m_stick = new Joystick(0);
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	ThreadVideoCapture video_capture_instance;
	Thread VideoCaptureThreadHandle;

	ThreadVisionProcessor vision_processor_instance;
	Thread VisionProcessorThreadHandle;

	ThreadArduinoGyro arduino_thread_instance;
	Thread arduinoThreadHandle;

	ThreadLIDAR lidar_instance;
	Thread lidarThreadHandle;

	
	public Robot() 
	{
		m_robotDrive.setExpiration(0.1);
		lidar_instance = new ThreadLIDAR();
		lidarThreadHandle = new Thread(lidar_instance);
		lidarThreadHandle.start();

		video_capture_instance = new ThreadVideoCapture();
		VideoCaptureThreadHandle = new Thread(video_capture_instance);
		VideoCaptureThreadHandle.start();
		video_capture_instance.Enable(); // begin getting frames.

		vision_processor_instance = new ThreadVisionProcessor(video_capture_instance, VideoCaptureThreadHandle, this); // pass
																														// in

		VisionProcessorThreadHandle = new Thread(vision_processor_instance);
		VisionProcessorThreadHandle.start();

		arduino_thread_instance = new ThreadArduinoGyro();
		arduinoThreadHandle = new Thread(arduino_thread_instance);
		arduinoThreadHandle.start();		
	} //Robot() constructor

	@Override
	public void robotInit() 
	{
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto modes", m_chooser);
		
	    leftDriveMotorLowPassFilter = new LowPassFilter(125); // prevent motors tearing gears apart
	    rightDriveMotorLowPassFilter = new LowPassFilter(125);

		
		
		leftDriveTalon13Follower.follow(leftDriveTalon12Driver);
		leftDriveTalon11Follower.follow(leftDriveTalon12Driver);
		
		rightDriveTalon20Follower.follow(rightDriveTalon19Driver);
		rightDriveTalon18Follower.follow(rightDriveTalon19Driver);
	
	} //robotInit()
	
	

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 *
	 * <p>If you wanted to run a similar autonomous mode with an IterativeRobot
	 * you would write:
	 *
	 * <blockquote><pre>{@code
	 * Timer timer = new Timer();
	 *
	 * // This function is run once each time the robot enters autonomous mode
	 * public void autonomousInit() {
	 *     timer.reset();
	 *     timer.start();
	 * }
	 *
	 * // This function is called periodically during autonomous
	 * public void autonomousPeriodic() {
	 * // Drive for 2 seconds
	 *     if (timer.get() < 2.0) {
	 *         myRobot.drive(-0.5, 0.0); // drive forwards half speed
	 *     } else if (timer.get() < 5.0) {
	 *         myRobot.drive(-1.0, 0.0); // drive forwards full speed
	 *     } else {
	 *         myRobot.drive(0.0, 0.0); // stop robot
	 *     }
	 * }
	 * }</pre></blockquote>
	 */
	@Override
	public void autonomous() 
	{
		String autoSelected = m_chooser.getSelected();
		// String autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

		// MotorSafety improves safety when motors are updated in loops
		// but is disabled here because motor updates are not looped in
		// this autonomous mode.
		m_robotDrive.setSafetyEnabled(false);

		switch (autoSelected) 
		{
			case kCustomAuto:
				// Spin at half speed for two seconds
				m_robotDrive.arcadeDrive(0.0, 0.5);
				Timer.delay(2.0);

				// Stop robot
				m_robotDrive.arcadeDrive(0.0, 0.0);
				break;
			case kDefaultAuto:
			default:
				// Drive forwards for two seconds
				m_robotDrive.arcadeDrive(-0.5, 0.0);
				Timer.delay(2.0);

				// Stop robot
				m_robotDrive.arcadeDrive(0.0, 0.0);
				break;
		}
	}

	/**
	 * Runs the motors with arcade steering.
	 *
	 * <p>If you wanted to run a similar teleoperated mode with an IterativeRobot
	 * you would write:
	 *
	 * <blockquote><pre>{@code
	 * // This function is called periodically during operator control
	 * public void teleopPeriodic() {
	 *     myRobot.arcadeDrive(stick);
	 * }
	 * }</pre></blockquote>
	 */
	@Override
	public void operatorControl() 
	{
		m_robotDrive.setSafetyEnabled(true);
		
		while (isOperatorControl() && isEnabled()) 
		{
			/* double leftYstick = _joy.getAxis(AxisType.kX); //.getY();
			 _talon.set(ControlMode.PercentOutput, leftYstick);  //left drive
			_talon11.set(ControlMode.PercentOutput, leftYstick);
			_talon13.set(ControlMode.PercentOutput, leftYstick);
			
			
			_talon18.set(ControlMode.PercentOutput, leftYstick); //right drive
			_talon19.set(ControlMode.PercentOutput, leftYstick);
			_talon20.set(ControlMode.PercentOutput, leftYstick);

			
			_talon16.set(ControlMode.PercentOutput, leftYstick);
			*/

			
			
		    double combined = driverStick.getRawAxis(3) * -1 + driverStick.getAxis(AxisType.kZ);
		    
		    mDriverRobotSpeed = leftDriveMotorLowPassFilter.calculate(combined);
		    // Calculate robot turn direction from the left drive joystick's x-axis
		    // (left-right)
		    mDriverRobotTurnDirection = rightDriveMotorLowPassFilter.calculate(driverStick.getAxis(AxisType.kX));
		    
		    //mRobot.mRobotSpeed = mDriverRobotSpeed;

		    // Last move robot. Let the robotdrive class handle the driving aspect of
		    // the robot

		    m_robotDrive.arcadeDrive(mDriverRobotSpeed, mDriverRobotTurnDirection); // move robot			
			
			
			
			//m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());

			SendDataToSmartDashboard();
			Timer.delay(0.005); // wait for a motor update time
		} // while isEnabled
		
	
	}

	
	void SendDataToSmartDashboard() 
	{
		long deltat = System.currentTimeMillis() - mLastDashboardUpdateTime;
		if (deltat > 1000) {
			SmartDashboard.putNumber("AUTOTARGET XPOS: ", vision_processor_instance.cregions.mXGreenLine);
			SmartDashboard.putNumber("Auto TARGET Enabled: ", vision_processor_instance.cregions.mTargetVisible);
			//SmartDashboard.putNumber("SHOOTER MotorOut:", mShooterController.motorOutput);
			//SmartDashboard.putNumber("SHOOTER RPM", mShooterController.shooterCANTalon.getSpeed());
			//SmartDashboard.putNumber("SHOOTER TARGET RPM:", mShooterController.targetRPM);
			//SmartDashboard.putNumber("SHOOTER ERROR:", mShooterController.shooterCANTalon.getClosedLoopError());
			SmartDashboard.putNumber("XCENTER", vision_processor_instance.cregions.mXGreenLine); // .lastXCenter);
			SmartDashboard.putNumber("NumContours:", vision_processor_instance.cregions.mContours.size());
			SmartDashboard.putNumber("MAPPED:", vision_processor_instance.lastMapped);
			mLastDashboardUpdateTime = System.currentTimeMillis();
			SmartDashboard.putNumber("LAST HEADING:", arduino_thread_instance.lastHeading);
			SmartDashboard.putNumber("LIDAR Angle:", lidar_instance.lastAngle);
			SmartDashboard.putNumber("LIDAR SS", lidar_instance.lastSignalStrength);
			SmartDashboard.putNumber("LIDAR distance", lidar_instance.lastDistance);
			SmartDashboard.putNumber("LIDAR status:", lidar_instance.lastStatus);
			SmartDashboard.putString("LIDAR LAST ERROR", lidar_instance.lastError);
			SmartDashboard.putString("LIDARMessage:", lidar_instance.lastMessage);
			SmartDashboard.putString("GYRO Last Error", arduino_thread_instance.lastError);
			SmartDashboard.putString("GYRO Message", arduino_thread_instance.lastMessage);
		}
	} // SendDataToSmartDashboard

	
	
	
	
	
	
	/**
	 * Runs during test mode.
	 */
	@Override
	public void test() {
	}
	
	
	public static class InputSystem {
		// Control stick buttons
		public static boolean Y_Button_Control_Stick = false; //
		public static boolean A_Button_Control_Stick = false;
		public static boolean X_Button_Control_Stick = false;
		public static boolean X_Button_Control_Stick_Prev = false;
		public static boolean B_Button_Control_Stick = false;
		public static boolean RB_Button_Control_Stick = false; // r
		public static boolean RB_Button_Control_Stick_Prev = false;
		public static boolean Start_Button_Control_Stick = false;
		public static boolean Start_Button_Control_Stick_Prev = false;
		public static boolean Start_Button_Control_Stick_Once = false;
		public static boolean Dpad_Up_Control_Stick = false;
		public static boolean Dpad_Up_Control_Stick_Prev = false;
		public static boolean Dpad_Up_Control_Stick_Pressed_Once = false;
		public static boolean Dpad_Up_Control_Stick_Released_Once = false;
		public static boolean Dpad_Down_Control_Stick = false;
		public static boolean Dpad_Down_Control_Stick_Prev = false;
		public static boolean Dpad_Down_Control_Stick_Pressed_Once = false;
		public static boolean Dpad_Down_Control_Stick_Released_Once = false;
		public static int Dpad_Angle_Control_Stick = 0;
		public static double RT_Button_Control_Stick = 0.0;
		public static double RT_Button_Control_Stick_Prev = 0.0;
		public static boolean RT_Button_Control_Stick_Pressed_Once = false;
		public static double LT_Button_Control_Stick = 0.0;
		public static double LT_Button_Control_Stick_Prev = 0.0;
		public static boolean LT_Button_Control_Stick_Pressed_Once = false;

		// Driver stick buttons
		public static boolean Y_Button_Driver_Stick = false;
		public static boolean A_Button_Driver_Stick = false;
		public static boolean X_Button_Driver_Stick = false; //
		public static boolean B_Button_Driver_Stick = false;
		public static boolean B_Button_Driver_Stick_Prev = false;
		public static boolean RB_Button_Driver_Stick = false; //
		public static boolean RB_Button_Driver_Stick_Prev = false;
		public static boolean Start_Button_Driver_Stick = false; // small black
																	// button on
																	// front
		public static boolean Start_Button_Driver_Stick_Prev = false;
		public static boolean Start_Button_Driver_Stick_Once = false;
		public static boolean Back_Button_Control_Stick = false;
		public static boolean Back_Button_Control_Stick_Prev = false;
		public static boolean Back_Button_Control_Stick_Once = false;

		public static boolean Turret_Limit_Switch = false;

		public static void ReadAllInput(Joystick driverstk, Joystick controlstk) {
			Y_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_Y_BUTTON);
			A_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_A_BUTTON);
			X_Button_Control_Stick_Prev = X_Button_Control_Stick;
			X_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_X_BUTTON);
			B_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_B_BUTTON);
			RB_Button_Control_Stick_Prev = RB_Button_Control_Stick;
			RB_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_RIGHT_BACK_BUTTON);
			Start_Button_Control_Stick_Prev = Start_Button_Control_Stick;
			Start_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_SMALL_START_BUTTON);
			Start_Button_Control_Stick_Once = Start_Button_Control_Stick_Prev == false
					&& Start_Button_Control_Stick == true;
			Back_Button_Control_Stick_Prev = Back_Button_Control_Stick;
			Back_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_SMALL_BACK_BUTTON);
			Back_Button_Control_Stick_Once = Back_Button_Control_Stick_Prev == false
					&& Back_Button_Control_Stick == true;
			Dpad_Angle_Control_Stick = controlstk.getPOV(IOMapping.CONTROL_DPAD);
			Dpad_Up_Control_Stick_Prev = Dpad_Up_Control_Stick;
			Dpad_Up_Control_Stick = Dpad_Angle_Control_Stick >= 315 || Dpad_Angle_Control_Stick <= 45;
			Dpad_Up_Control_Stick_Pressed_Once = Dpad_Up_Control_Stick_Prev == false && Dpad_Up_Control_Stick == true;
			Dpad_Up_Control_Stick_Released_Once = Dpad_Up_Control_Stick_Prev == true && Dpad_Up_Control_Stick == false;
			Dpad_Down_Control_Stick_Prev = Dpad_Down_Control_Stick;
			Dpad_Down_Control_Stick = Dpad_Angle_Control_Stick >= 135 && Dpad_Angle_Control_Stick <= 225;
			Dpad_Down_Control_Stick_Pressed_Once = Dpad_Down_Control_Stick_Prev == false
					&& Dpad_Down_Control_Stick == true;
			Dpad_Down_Control_Stick_Released_Once = Dpad_Down_Control_Stick_Prev == true
					&& Dpad_Down_Control_Stick == false;
			RT_Button_Control_Stick_Prev = RT_Button_Control_Stick;
			RT_Button_Control_Stick = controlstk.getRawAxis(3);
			RT_Button_Control_Stick_Pressed_Once = Math.abs(RT_Button_Control_Stick_Prev) < 0.1
					&& Math.abs(RT_Button_Control_Stick) >= 0.1;
			LT_Button_Control_Stick_Prev = LT_Button_Control_Stick;
			LT_Button_Control_Stick = controlstk.getAxis(AxisType.kZ);
			LT_Button_Control_Stick_Pressed_Once = Math.abs(LT_Button_Control_Stick_Prev) < 0.1
					&& Math.abs(LT_Button_Control_Stick) >= 0.1;

			Y_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_Y_BUTTON);
			A_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_A_BUTTON);
			X_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_X_BUTTON);
			B_Button_Driver_Stick_Prev = B_Button_Driver_Stick;
			B_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_B_BUTTON);
			RB_Button_Driver_Stick_Prev = RB_Button_Driver_Stick;
			RB_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_RIGHT_BACK_BUTTON);
			Start_Button_Driver_Stick_Prev = Start_Button_Driver_Stick;
			Start_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_SMALL_START_BUTTON);
			Start_Button_Driver_Stick_Once = Start_Button_Driver_Stick_Prev == false
					&& Start_Button_Driver_Stick == true;
		} // ReadAllInput
	} // public static inputsystem class

	
}
