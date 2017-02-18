
package org.usfirst.frc.team5000.robot;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
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

	static enum DriveState {
		Stopped, Forward, Reverse, Left, Right, Turn
	};

	static enum AutoStep {
		Start, Step1, Step2, Step3, Step4, Stop;

		public AutoStep next() {
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

		public String toString() {
			switch (this) {
			case Start:
				return "Start";
			case Step1:
				return "Step1";
			case Step2:
				return "Step2";
			case Step3:
				return "Step3";
			case Step4:
				return "Step4";
			default:
				return "Stop";
			}
		}
	};

	// Constants

	static final double FORWARD_WINCH_SPEED = -0.7;
	static final double REVERSE_WINCH_SPEED = 0.6;
	static final double FORWARD_DOOR_SPEED = 0.5; // 2/15 was 0.7
	static final double REVERSE_DOOR_SPEED = -0.5; // 2/15 was -0.7
	static final boolean USE_MECANUM_DRIVE = true;
	static final int DRIVE_DIRECTION_SWITCH_BUTTON = 2;
	static final int OPEN_DOOR_BUTTON = 3;
	static final int CLOSE_DOOR_BUTTON = 2;
	static final int WINCH_UP_BUTTON = 1;
	static final int WINCH_QUICK_RELEASE_BUTTON = 1;
	static final long QUICK_RELEASE_TIME = 125;
	static final String LEFT_AUTO = "Left";
	static final String CENTER_AUTO = "Center";
	static final String RIGHT_AUTO = "Right";
	static final double Kp = 0.03;

	// Variables
	SendableChooser<String> chooser;
	String autoSelected;
	AutoStep autoStep = AutoStep.Start;
	boolean initAutoStep = true;
	double targetSpeed = 0;
	long targetTime = 0;
	double targetAngle = 0;
	double targetTurningSpeed = 0;
	double currentAngle = 0;
	double angularDistance = 0;
	boolean watchForReflectiveStrips = false;
	DriveState driveState = DriveState.Stopped;
	double baseDriveLevel = 0.2;
	double baseTwistLevel = 0.37;

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
	boolean leftIRSensor, rightIRSensor;
	long quickReleaseEndTime = 0;

	Gyro gyro;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {

		chooser = new SendableChooser<String>();
		chooser.addObject("Left Auto", LEFT_AUTO);
		chooser.addObject("Center Auto", CENTER_AUTO);
		chooser.addDefault("Right Auto", RIGHT_AUTO);
		SmartDashboard.putData("Auto choices2 ", chooser);

		driveJoystick = new Joystick(0);
		doorJoystick = new Joystick(1);
		driveJoystickButtons = new HHJoystickButtons(driveJoystick, 10);
		doorJoystickButtons = new HHJoystickButtons(doorJoystick, 10);

		driveCimLF = new CANTalon(2);
		driveCimLR = new CANTalon(3);
		driveCimRF = new CANTalon(1);
		driveCimRR = new CANTalon(0);

		driveCimLF.setInverted(true);
		if (USE_MECANUM_DRIVE) {
			driveCimLR.setInverted(true);
		}

		if (USE_MECANUM_DRIVE) {
			driveTrain = new RobotDrive(driveCimLF, driveCimLR, driveCimRF, driveCimRR);
		} else {
			driveTrain = new RobotDrive(driveCimRF, driveCimLR);
		}

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

                //		gyro = new AnalogGyro(0);
                //		gyro = new ADXRS450_Gyro(0);
		gyro = new ADXRS450_Gyro();
		// gyro.calibrate();

		SmartDashboard.putString("Camera 2", driveDirection == MotorState.Forward ? "Forward" : "Reverse");
		SmartDashboard.putString("Camera 1", driveDirection == MotorState.Reverse ? "Forward" : "Reverse");

		SmartDashboard.putString("Auto Step ", AutoStep.Start.toString());

		leftIRSensor = IR_Sensor_L.get();
		rightIRSensor = IR_Sensor_R.get();

		SmartDashboard.putString("Left IR  ", Boolean.toString(leftIRSensor));
		SmartDashboard.putString("Right IR ", Boolean.toString(rightIRSensor));

		SmartDashboard.putNumber("Base Drive Level ", baseDriveLevel);
		SmartDashboard.putNumber("Base Twist Level ", baseTwistLevel);
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
		autoSelected = chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);

		gyro.reset();

		autoStep = AutoStep.Start;
		initializeForNextStep();
		driveDirection = MotorState.Reverse;

		SmartDashboard.putString("Auto Step ", autoStep.toString());
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		currentAngle = gyro.getAngle();

		String strDouble = String.format("%.2f", currentAngle);

		SmartDashboard.putString("Gyro Angle2 ", strDouble);

		angularDistance = getAngularDistanceFromTarget(currentAngle, targetAngle);

		SmartDashboard.putNumber("Target Angle ", targetAngle);
		SmartDashboard.putNumber("D ", angularDistance);
		
		leftIRSensor = IR_Sensor_L.get();
		rightIRSensor = IR_Sensor_R.get();

		SmartDashboard.putString("Left IR  ", Boolean.toString(leftIRSensor));
		SmartDashboard.putString("Right IR ", Boolean.toString(rightIRSensor));

		boolean incrementStep = false;

		if (targetTime == 0 && targetTurningSpeed == 0) {
			incrementStep = true;
		} else if (0 < targetTime && targetTime <= System.currentTimeMillis()) {
			incrementStep = true;
		} else if (targetTurningSpeed > 0 && Math.abs(angularDistance) < 1) {
			incrementStep = true;
		} else if (watchForReflectiveStrips && leftIRSensor && rightIRSensor) {
			incrementStep = true;
		}

		if (incrementStep) {
			initializeForNextStep();
			autoStep = autoStep.next();

			SmartDashboard.putString("Auto Step ", autoStep.toString());
		}

		switch (autoSelected) {
		case LEFT_AUTO:
			leftAutoProgram();
			break;

		default:
		case CENTER_AUTO:
			centerAutoProgram();
			break;

		case RIGHT_AUTO:
			rightAutoProgram();
			break;
		}

		doStep();
	}

	void leftAutoProgram() {
		switch (autoStep) {
		case Start:
			break;

		case Step1:
			driveForward(0.10, 2000);
			break;

		case Step2:
			driveReverse(0.10, 2000);
			break;

		case Step3:
			stop();
			break;

		case Step4:
			stop();
			break;

		case Stop:
			stop();
			break;
		}
	}

	void centerAutoProgram() {
		switch (autoStep) {
		case Start:
			break;

		case Step1:
			slideRight(0.10, 2000);
			break;

		case Step2:
			slideLeft(0.10, 2000);
			break;

		case Step3:
			stop();
			break;

		case Step4:
			stop();
			break;

		case Stop:
			stop();
			break;
		}
	}

	void rightAutoProgram() {
		switch (autoStep) {
		case Start:
			break;

		case Step1:
			turnRight(0.10, 120);
			watchForReflectiveStrips = true;
			break;

		case Step2:
			turnLeft(0.10, 120);
//			watchForReflectiveStrips = true;
			break;

		case Step3:
			stop();
			break;

		case Step4:
			stop();
			break;

		case Stop:
			stop();
			break;
		}
	}

	void doStep() {

		double x = 0;
		double y = 0;
		double t = 0;
		double b = 0;

		switch (driveState) {

		case Forward:
			y = targetSpeed;
//			t = -angularDistance * Kp;
			break;

		case Reverse:
			y = -targetSpeed;
//			t = -angularDistance * Kp;
			break;

		case Left:
			x = -targetSpeed;
//			t = -angularDistance * Kp;
			b = SmartDashboard.getNumber("Base Drive Level ", 0);
			break;

		case Right:
			x = targetSpeed;
//			t = -angularDistance * Kp;
			b = SmartDashboard.getNumber("Base Drive Level ", 0);
			break;

		case Turn:
			t = getTurningSpeedFromAngularDistance(angularDistance);
			break;

		case Stopped:
		default:
			break;
		}

		baseTwistLevel = SmartDashboard.getNumber("Base Twist Level ", 0);

		b = Math.max( 0.0, b );
		b = Math.min( 1.0, b );

		baseTwistLevel = Math.max( 0.0, baseTwistLevel );
		baseTwistLevel = Math.min( 1.0, baseTwistLevel );

		if (x < 0) {
			x = ((1.0 - b) * x) - b;
		} else if (x > 0) {
			x = ((1.0 - b) * x) + b;
		}

		if (y < 0) {
			y = ((1.0 - b) * y) - b;
		} else if (y > 0) {
			y = ((1.0 - b) * y) + b;
		}

		if (t < 0) {
			t = ((1.0 - baseTwistLevel) * t) - baseTwistLevel;
		} else if (t > 0) {
			t = ((1.0 - baseTwistLevel) * t) + baseTwistLevel;
		}

		SmartDashboard.putNumber("X ", x);
		SmartDashboard.putNumber("Y ", y);
		SmartDashboard.putNumber("T ", t);

		if (USE_MECANUM_DRIVE) {
			double gyroAngle = (driveDirection == MotorState.Forward) ? 180 : 0;
			driveTrain.mecanumDrive_Cartesian(x, y, t, gyroAngle);
		} else {
			driveTrain.arcadeDrive(-x, t);
		}
	}

	void initializeForNextStep() {
		targetTime = 0;
		targetSpeed = 0;
		targetAngle = 0;
		targetTurningSpeed = 0;
		watchForReflectiveStrips = false;
		initAutoStep = true;
	}

	void driveForward(double speed, long time) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Forward;
			targetSpeed = speed;
			targetTime = getTargetTime(time);
			targetAngle = currentAngle;
		}
	}

	void driveReverse(double speed, long time) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Reverse;
			targetSpeed = speed;
			targetTime = getTargetTime(time);
			targetAngle = currentAngle;
		}
	}

	void slideLeft(double speed, long time) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Left;
			targetSpeed = speed;
			targetTime = getTargetTime(time);
			targetAngle = currentAngle;
		}
	}

	void slideRight(double speed, long time) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Right;
			targetSpeed = speed;
			targetTime = getTargetTime(time);
			targetAngle = currentAngle;
		}
	}

	void turnLeft(double speed, double angle) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Turn;
			targetTurningSpeed = speed;
			targetAngle = getTargetAngle(currentAngle, -angle);
		}
	}

	void turnRight(double speed, double angle) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Turn;
			targetTurningSpeed = speed;
			targetAngle = getTargetAngle(currentAngle, angle);
		}
	}

	void stop() {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Stopped;
			targetTime = 0;
			targetSpeed = 0;
			targetAngle = 0;
			targetTurningSpeed = 0;
		}
	}

	long getTargetTime(long delta) {
		return System.currentTimeMillis() + delta;
	}

	double getTargetAngle(double currentAngle, double delta) {

		return (currentAngle + delta) % 360;
	}

	double getAngularDistanceFromTarget(double currentAngle, double targetAngle) {

		double delta = targetAngle - currentAngle;

		if (delta > 180)
			delta -= 360;
		if (delta < -180)
			delta += 360;

		return delta;
	}

	double getTurningSpeedFromAngularDistance(double delta) {

		double t = (delta < 0) ? -targetTurningSpeed : targetTurningSpeed;

		double d = Math.abs(delta);

		if (d < 10) {
			t = t * (d / 10);
		}

		return t;
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (driveState != DriveState.Stopped) {
			if (USE_MECANUM_DRIVE) {
				driveTrain.mecanumDrive_Cartesian(0, 0, 0, 180);
			} else {
				driveTrain.stopMotor();
			}
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

		currentAngle = gyro.getAngle();

		String strDouble = String.format("%.2f", currentAngle);

		SmartDashboard.putString("Gyro Angle2 ", strDouble);

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

			double gyroAngle = (driveDirection == MotorState.Forward) ? 180 : 0;

			double x = driveJoystick.getX();
			double y = driveJoystick.getY();
			double t = driveJoystick.getTwist();

			SmartDashboard.putNumber("X ", x);
			SmartDashboard.putNumber("Y ", y);
			SmartDashboard.putNumber("T ", t);
			SmartDashboard.putNumber("D ", 0);

			driveTrain.mecanumDrive_Cartesian(x * Math.abs(x), -(y * Math.abs(y)), -(t * Math.abs(t)), gyroAngle);
			// changed from x*x,y*y,t*t, orientation changed to 180 from 0 2/8
			// JF
		} else {
			driveTrain.arcadeDrive(driveJoystick);
		}
	}

	void doorPeriodic() {
		if (doorJoystickButtons.isPressed(CLOSE_DOOR_BUTTON)) {
			closeDoors();
		} else if (doorJoystickButtons.isPressed(OPEN_DOOR_BUTTON)) {
			openDoors();
		}

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

	void openDoors() {
		doorState = MotorState.Reverse;

		setDoorStatus();
	}

	void closeDoors() {
		doorState = MotorState.Forward;

		setDoorStatus();
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
