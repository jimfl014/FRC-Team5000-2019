
package org.usfirst.frc.team5000.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.*; //Added 2/8 JF to support Boolean to String method
import java.lang.Math;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	// Enums
	static enum MotorState {
		Stopped, Forward, Reverse
	};

	static enum AutoState {
		Start, Step1, Step2, Step3, Step4, Stop;

		public AutoState next() {
			switch (this) {
			case Start:
				return Step1;
			case Step1:
				return Step2;
			case Step2:
				return Step3;
			case Step3:
				return Step4;
			default:
				return Stop;
			}
		}
	};

	static enum DriveState {
		Stopped, Forward, Reverse, TurnRight, TurnLeft
	};

	// Constants

	static final double FORWARD_WINCH_SPEED = -0.7;
	static final double REVERSE_WINCH_SPEED = 0.6;
	static final double FORWARD_DOOR_SPEED = 0.5; //2/15 was 0.7
	static final double REVERSE_DOOR_SPEED = -0.5; //2/15 was -0.7
	static final boolean USE_MECANUM_DRIVE = true;
	static final int DRIVE_DIRECTION_SWITCH_BUTTON = 2;
	static final int OPEN_DOOR_BUTTON = 3;
	static final int CLOSE_DOOR_BUTTON = 2;
	static final int WINCH_UP_BUTTON = 1;
	static final int WINCH_QUICK_RELEASE_BUTTON = 1;
	static final long QUICK_RELEASE_TIME = 125;
	final String DEFAULT_AUTO = "Default";
	final String CUSTOM_AUTO = "My Auto";

	// Variables
	String autoSelected;
	AutoState autoState;
	long autoStepEndTime = 0;
	DriveState driveState = DriveState.Stopped;
	SendableChooser chooser;

	CANTalon driveCimLF, driveCimLR, driveCimRF, driveCimRR;
	Spark door, winch;
	Joystick driveJoystick, doorJoystick;
	HHJoystickButtons driveJoystickButtons, doorJoystickButtons;
	RobotDrive driveTrain;
	MotorState driveDirection = MotorState.Forward;
	MotorState winchState = MotorState.Stopped;
	MotorState doorState = MotorState.Forward;

	CameraServer cameraServer;
	PowerDistributionPanel pdp;

	DigitalInput IR_Sensor_L, IR_Sensor_R; // Added 2/8 JF
	long quickReleaseEndTime = 0;

	AnalogGyro gyro;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {

		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", DEFAULT_AUTO);
		chooser.addObject("Test Auto", CUSTOM_AUTO);
		SmartDashboard.putData("Auto choices", chooser);

		autoState = AutoState.Start;

		driveJoystick = new Joystick(0);
		doorJoystick = new Joystick(1);
		driveJoystickButtons = new HHJoystickButtons(driveJoystick, 10);
		doorJoystickButtons = new HHJoystickButtons(doorJoystick, 10);

		driveCimLF = new CANTalon(2);
		driveCimLR = new CANTalon(3);
		driveCimRF = new CANTalon(1);
		driveCimRR = new CANTalon(0);

		driveCimLF.setInverted(true);
		driveCimLR.setInverted(true);

		driveTrain = new RobotDrive(driveCimLF, driveCimLR, driveCimRF, driveCimRR);
		// driveTrain = new
		// RobotDrive(driveCimLF,driveCimLR,driveCimRF,driveCimRR);
		door = new Spark(0);
		winch = new Spark(1);

		cameraServer = CameraServer.getInstance();
		// camera1.setQuality(50);
		cameraServer.startAutomaticCapture("cam0", 0);
		cameraServer.startAutomaticCapture("cam1", 1);

		pdp = new PowerDistributionPanel();
		pdp.startLiveWindowMode();

		IR_Sensor_L = new DigitalInput(0); // Added 2/8 JF
		IR_Sensor_R = new DigitalInput(1);

		gyro = new AnalogGyro(0);

		SmartDashboard.putString("Camera 2", driveDirection == MotorState.Forward ? "Forward" : "Reverse");
		SmartDashboard.putString("Camera 1", driveDirection == MotorState.Reverse ? "Forward" : "Reverse");
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = (String) chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// DEFAULT_AUTO);
		System.out.println("Auto selected: " + autoSelected);
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		
		autoState = AutoState.Start;
		autoStepEndTime = 0;
		driveDirection = MotorState.Forward;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		switch (autoSelected) {
		case CUSTOM_AUTO:
	
			// break;
		case DEFAULT_AUTO:
		default:
			customAutoPeriodic();
			// Put default auto code here
			break;
		}
	}

	void customAutoPeriodic() {
		switch (autoState) {
		case Start:
			if (autoStepEndTime == 0) {
				driveState = DriveState.Forward;
				autoStepEndTime = System.currentTimeMillis() + 2000;
			}
			break;
			
		case Step1:
			if (autoStepEndTime == 0) {
				driveState = DriveState.Stopped;
			}
			break;
	
		case Step2:
			if (autoStepEndTime == 0) {
				driveState = DriveState.Stopped;
			}
			break;
	
		case Step3:
			if (autoStepEndTime == 0) {
				driveState = DriveState.Stopped;
			}
			break;
	
		case Step4:
			if (autoStepEndTime == 0) {
				driveState = DriveState.Stopped;
			}
			break;
	
		case Stop:
			autoStepEndTime = 0;
			driveState = DriveState.Stopped;
			break;
		}

		if (System.currentTimeMillis() < autoStepEndTime) {

			double x = 0;
			double y = 0;
			double t = 0;

			switch (driveState) {
			case Stopped:
				break;

			case Forward:
				x = 0.7;
				break;

			case Reverse:
				x = -0.7;
				break;

			case TurnRight:
				t = 0.5;
				break;

			case TurnLeft:
				t = -0.5;
				break;
			}

			double gyroAngle = (driveDirection == MotorState.Forward) ? 180 : 0;

			driveTrain.mecanumDrive_Cartesian(x, y, t, gyroAngle);

		} else {
			autoStepEndTime = 0;
			autoState = autoState.next();
		}
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autoState != AutoState.Stop) {
			driveTrain.mecanumDrive_Cartesian(0, 0, 0, 180);
		}

		gyro.reset();
		doorState = MotorState.Forward;
		setDoorStatus();
	}

	/**
	 * This function is called periodically during operator control
	 */
	// @Override
	public void teleopPeriodic() {

		pdp.updateTable();

		driveJoystickButtons.updateState();
		doorJoystickButtons.updateState();

		drivePeriodic();
		doorPeriodic();
		winchPeriodic();

		// Section below added 2/8 JF to get and display IR sensor data
		String pin_Status_L = new Boolean(IR_Sensor_L.get()).toString();
		String pin_Status_R = new Boolean(IR_Sensor_R.get()).toString();

		SmartDashboard.putString("Left IR  ", pin_Status_L);
		SmartDashboard.putString("Right IR ", pin_Status_R);

		double angle = gyro.getAngle();
		
		SmartDashboard.putNumber("Gyro Angle ", angle);
	}

	/**
	 * This function is called periodically during test mode
	 */
	// @Override
	public void testPeriodic() {
	}

	void drivePeriodic() {
		if (USE_MECANUM_DRIVE) {

			if (driveJoystickButtons.isPressed(DRIVE_DIRECTION_SWITCH_BUTTON)) {
				if (driveDirection == MotorState.Forward) {
					driveDirection = MotorState.Reverse;
				} else {
					driveDirection = MotorState.Forward;
				}

				SmartDashboard.putString("Camera 2", driveDirection == MotorState.Forward ? "Forward" : "Reverse");
				SmartDashboard.putString("Camera 1", driveDirection == MotorState.Reverse ? "Forward" : "Reverse");
			}

			double d = driveDirection == MotorState.Forward ? 1.0 : -1.0;
			double gyroAngle = (driveDirection == MotorState.Forward) ? 180 : 0;

			double x = driveJoystick.getX();
			double y = driveJoystick.getY();
			double t = driveJoystick.getTwist();

			driveTrain.mecanumDrive_Cartesian(x * Math.abs(x), -(y * Math.abs(y)), -(t * Math.abs(t)), gyroAngle);
			// changed from x*x,y*y,t*t, orientation changed to 180 from 0 2/8
			// JF
		} else {
			driveTrain.arcadeDrive(driveJoystick);
		}
	}

	void doorPeriodic() {
		if (doorJoystickButtons.isPressed(CLOSE_DOOR_BUTTON)) {
			doorState = MotorState.Forward;
		} else if (doorJoystickButtons.isPressed(OPEN_DOOR_BUTTON)) {
			doorState = MotorState.Reverse;
		}
		
		setDoorStatus();

		SmartDashboard.putNumber("Door Current", pdp.getCurrent(0));
	}

	void winchPeriodic() {
		if (doorJoystickButtons.isPressed(WINCH_UP_BUTTON)) {
			if (winchState == MotorState.Stopped) {
				winchState = MotorState.Forward;
			} else if (winchState == MotorState.Forward) {
				winchState = MotorState.Stopped;
			}
		}
		if (driveJoystickButtons.isPressed(WINCH_QUICK_RELEASE_BUTTON)) {
			quickReleaseEndTime = System.currentTimeMillis() + QUICK_RELEASE_TIME;
			winchState = MotorState.Forward;
		}
		if (0 < quickReleaseEndTime && quickReleaseEndTime <= System.currentTimeMillis()) {
			winchState = MotorState.Stopped;
			quickReleaseEndTime = 0;
		}
		// below commented out 2/8 JF
		/*
		 * if (doorJoystickButtons.isPressed(WINCH_DOWN_BUTTON)) { if
		 * (winchState == MotorState.Stopped) { winchState = MotorState.Reverse;
		 * } else if (winchState == MotorState.Reverse) { winchState =
		 * MotorState.Stopped; } }
		 */

		String winchStatus;

		if (winchState == MotorState.Forward) {
			winch.set(FORWARD_WINCH_SPEED);
			winchStatus = "Forward";
		} else if (winchState == MotorState.Reverse) {
			winch.set(REVERSE_WINCH_SPEED);
			winchStatus = "Reverse";
		} else {
			winch.stopMotor();
			winchStatus = "Stopped";
		}
		SmartDashboard.putNumber("Winch Current", pdp.getCurrent(15));
		SmartDashboard.putString("Winch Status", winchStatus);
		/* Change # for real robot */
	}
	
	void setDoorStatus() {
		String doorStatus = "Stopped";

		if (doorState == MotorState.Stopped) {
			door.stopMotor();
			doorStatus = "Stopped";
		} else if (doorState == MotorState.Forward) {
			door.set(FORWARD_DOOR_SPEED);
			doorStatus = "Closed";
		} else if (doorState == MotorState.Reverse) {
			door.set(REVERSE_DOOR_SPEED);
			doorStatus = "Open";
		}

		SmartDashboard.putString("Door Status", doorStatus);
	}
}
