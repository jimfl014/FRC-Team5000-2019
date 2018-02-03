package org.usfirst.frc.team5000.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot implements PIDOutput {
    
	// Enums
	static enum ClawState {
		InitialOpen, Close, SecondOpen, Push
	}

	static enum MotorState {
		Stopped, Forward, Reverse
	};

	static enum DriveState {
		Stopped, Forward, Reverse, Left, Right, Turn
	};

	static enum AutoStep {
		Start, Step1, Step2, Step3, Step4, Step5, Step6, Step7, Step8, Step9, Step10, Stop;

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
			case Step4:
				return Step5;
			case Step5:
				return Step6;
			case Step6:
				return Step7;
			case Step7:
				return Step8;
			case Step8:
				return Step9;
			case Step9:
				return Step10;
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
			case Step5:
				return "Step5";
			case Step6:
				return "Step6";
			case Step7:
				return "Step7";
			case Step8:
				return "Step8";
			case Step9:
				return "Step9";
			case Step10:
				return "Step10";
			default:
				return "Stop";
			}
		}
	};

	// Constants

	static final double FORWARD_WINCH_SPEED = -0.7;
	static final double REVERSE_WINCH_SPEED = 0.6;
	static final double FORWARD_DOOR_SPEED = 0.3; // 2/15 was 0.7
	static final double REVERSE_DOOR_SPEED = -0.3; // 2/15 was -0.7
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

	static final int forwardChannel1 = 0; //this is open
	static final int reverseChannel1 = 1; //this is close
	static final int armsopen = 1;
	static final int armsclose = 2;
	static final int forwardChannel2 = 2; //this is out
	static final int reverseChannel2 = 3; //this is in
	static final long delaybeforepush = 100;
	static final long delaybeforeretract = 500;
	
	static final int resetrotate = 4;
	static final int rotatezero = 5;
	static final int rotateninety = 6;
	static final int rotateoneeighty = 7;
	static final int rotatetwoseventy = 8;
    
	static final int actuator1FWD = 2;
	static final int actuator1REV = 3;
	//static final int actuator2FWD = 4;
	//static final int actuator2REV = 5;
			
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	
	static final double kToleranceDegrees = 2.0f;

	// Variables
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

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
	double baseDriveLevel = 0.05;
	double baseTwistLevel = 0.15;
	long beforeTurnTime = 2300;
	long afterTurnTime = 2000;
	long centerDriveTime = 1800;
	boolean leftIRSensor, rightIRSensor;
    
	WPI_TalonSRX driveCimLF, driveCimLR, driveCimRF, driveCimRR;
	Joystick driveJoystick;
	HHJoystickButtons driveJoystickButtons;
	DifferentialDrive driveTrain;
	
	DoubleSolenoid arms, pusher;

	long firstdelay = 0;
	long seconddelay = 0;
    
	AHRS ahrs;
	
	PIDController turnController;
	double rotateToAngleRate;
	
	ClawState clawState;
    
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);

		SmartDashboard.putString("Auto Step ", AutoStep.Start.toString());
		
		driveCimLF = new WPI_TalonSRX(0);
		driveCimLR = new WPI_TalonSRX(1);
		driveCimRF = new WPI_TalonSRX(2);
		driveCimRR = new WPI_TalonSRX(3);

		SpeedControllerGroup right = new SpeedControllerGroup(driveCimRF,driveCimRR);
		SpeedControllerGroup left = new SpeedControllerGroup(driveCimLF,driveCimLR);
		
		driveTrain = new DifferentialDrive(right,left);
		driveJoystick = new Joystick(0);
		driveJoystickButtons = new HHJoystickButtons(driveJoystick, 10);
	
//		arms = new DoubleSolenoid(forwardChannel1, reverseChannel1);
//		pusher = new DoubleSolenoid(forwardChannel2, reverseChannel2);

		// ahrs = new AHRS(SerialPort.Port.kMXP.kUSB);
		ahrs = new AHRS(Port.kUSB);
		
		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0,  1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
	}

   /** Need to add teleopInit JF
    * 
    */
	@Override
	public void teleopInit() {
	}
		
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		// driveTrain.arcadeDrive(driveJoystick);
		
		driveJoystickButtons.updateState();
		
                //		driveTrain.arcadeDrive(driveJoystick.getY(), driveJoystick.getZ());
		
                //		openCloseClaw();
		
		SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
		SmartDashboard.putNumber("IMU_Angle", ahrs.getAngle());
		
		boolean rotateToAngle = false;
		
		if(driveJoystickButtons.isPressed(resetrotate)){
			ahrs.reset();
		}
                
		if (driveJoystickButtons.isPressed(rotatezero)){
			turnController.setSetpoint(0.0f);
			rotateToAngle = true;
		} else if (driveJoystickButtons.isPressed(rotateninety)){
			turnController.setSetpoint(90.0f);
			rotateToAngle = true;
		} else if (driveJoystickButtons.isPressed(rotateoneeighty)){
			turnController.setSetpoint(179.9f);
			rotateToAngle = true;
		} else if (driveJoystickButtons.isPressed(rotatetwoseventy)){
			turnController.setSetpoint(-90.0f);
			rotateToAngle = true;
		}

		double currentRotationRate;

		if (rotateToAngle){
			turnController.enable();
			currentRotationRate = rotateToAngleRate;
		} else {
			turnController.disable();
			currentRotationRate = driveJoystick.getTwist();
		}
		try {
			driveTrain.arcadeDrive(driveJoystick.getY(), currentRotationRate);
		} catch(RuntimeException ex) {
			DriverStation.reportError("Error communicating with drive system: "+ ex.getMessage(), true);
		}
	}
	
	void openCloseClaw() {
		
		switch (clawState) {
		case InitialOpen:
			arms.set(DoubleSolenoid.Value.kForward); //arms open
			pusher.set(DoubleSolenoid.Value.kReverse); //pusher retracted
			if (driveJoystickButtons.isPressed(armsclose)) {
				clawState = ClawState.Close;
			}
			break;
		case Close:
			arms.set(DoubleSolenoid.Value.kReverse); //arms close
			if (driveJoystickButtons.isPressed(armsopen)) {
				clawState = ClawState.SecondOpen;
				firstdelay = System.currentTimeMillis() + delaybeforepush;
			}
			break;
		case SecondOpen:
			arms.set(DoubleSolenoid.Value.kForward); // arms open
			if(firstdelay >= System.currentTimeMillis()){
				clawState = ClawState.Push;
				seconddelay = System.currentTimeMillis() + delaybeforeretract;
			}
			break;
		case Push:
			pusher.set(DoubleSolenoid.Value.kForward); //pusher out
			if(seconddelay >= System.currentTimeMillis()){
				clawState = ClawState.InitialOpen;
			}
			break;
		}
	}
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);

		autoStep = AutoStep.Start;
		SmartDashboard.putString("Auto Step ", autoStep.toString());
		initializeForNextStep();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

            //		currentAngle = gyro.getAngle();

            //		SmartDashboard.putString("Gyro Angle2 ", String.format("%.2f", currentAngle));

            //		angularDistance = getAngularDistanceFromTarget(currentAngle, targetAngle);

            //		SmartDashboard.putNumber("Target Angle ", targetAngle);
            //		SmartDashboard.putNumber("D ", angularDistance);

            //		leftIRSensor = irSensorLeft.get();
            //		rightIRSensor = irSensorRight.get();

            //		SmartDashboard.putString("Left IR  ", Boolean.toString(leftIRSensor));
            //		SmartDashboard.putString("Right IR ", Boolean.toString(rightIRSensor));

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

		long driveTime = 0;

		switch (autoStep) {
		case Start:
			break;

		case Step1:
			try {
				driveTime = Long.parseLong(SmartDashboard.getString("Before Turn Time ", "bad"));
			} catch (NumberFormatException e) {
				driveTime = beforeTurnTime;
			}

			driveForward(0.3, driveTime);
			break;

		case Step2:
			turnRight(0.10, 90);
			watchForReflectiveStrips = true;
			break;

		case Step3:
			try {
				driveTime = Long.parseLong(SmartDashboard.getString("After Turn Time ", "bad"));
			} catch (NumberFormatException e) {
				driveTime = afterTurnTime;
			}

			driveForward(0.2, driveTime);
			break;

		case Step4:
			pause(1000);
			openDoors();
			break;

		case Step5:
			driveReverse(0.3, 1000);

		case Stop:
		default:
			stop();
			closeDoors();
			break;
		}
	}

	void centerAutoProgram() {
		long driveTime = 0;

		switch (autoStep) {
		case Start:
			break;

		case Step1:
			try {
				driveTime = Long.parseLong(SmartDashboard.getString("Center Drive Time ", "bad"));
			} catch (NumberFormatException e) {
				driveTime = centerDriveTime;
			}
			driveForward( 0.05, driveTime );
			break;

		case Stop:
		default:
			stop();
			break;
		}
	}

	void rightAutoProgram() {
		long driveTime = 0;

		switch (autoStep) {
		case Start:
			break;

		case Step1:
			try {
				driveTime = Long.parseLong(SmartDashboard.getString("Before Turn Time ", "bad"));
			} catch (NumberFormatException e) {
				driveTime = beforeTurnTime;
			}

			driveForward(0.3, driveTime);
			break;

		case Step2:
			turnLeft(0.10, 90);
			watchForReflectiveStrips = true;
			break;

		case Step3:
			try {
				driveTime = Long.parseLong(SmartDashboard.getString("After Turn Time ", "bad"));
			} catch (NumberFormatException e) {
				driveTime = afterTurnTime;
			}

			driveForward(0.2, driveTime);
			break;

		case Step4:
			pause(1000);
			openDoors();
			break;

		case Step5:
			driveReverse(0.3, 1000);

		case Stop:
		default:
			stop();
			closeDoors();
			break;
		}
	}

	void doStep() {

		double x = 0;
		double y = 0;
		double t = 0;

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
			x = targetSpeed;
			//			t = -angularDistance * Kp;
			break;

		case Right:
			x = -targetSpeed;
			//			t = -angularDistance * Kp;
			break;

		case Turn:
			t = getTurningSpeedFromAngularDistance(angularDistance);
			break;

		case Stopped:
		default:
			break;
		}

		baseDriveLevel = SmartDashboard.getNumber("Base Drive Level ", 0);
		baseTwistLevel = SmartDashboard.getNumber("Base Twist Level ", 0);

		baseDriveLevel = Math.max( 0.0, baseDriveLevel );
		baseDriveLevel = Math.min( 1.0, baseDriveLevel );

		baseTwistLevel = Math.max( 0.0, baseTwistLevel );
		baseTwistLevel = Math.min( 1.0, baseTwistLevel );

		if (x < 0) {
			x = ((1.0 - baseDriveLevel) * x) - baseDriveLevel;
		} else if (x > 0) {
			x = ((1.0 - baseDriveLevel) * x) + baseDriveLevel;
		}

		if (y < 0) {
			y = ((1.0 - baseDriveLevel) * y) - baseDriveLevel;
		} else if (y > 0) {
			y = ((1.0 - baseDriveLevel) * y) + baseDriveLevel;
		}

		if (t < 0) {
			t = ((1.0 - baseTwistLevel) * t) - baseTwistLevel;
		} else if (t > 0) {
			t = ((1.0 - baseTwistLevel) * t) + baseTwistLevel;
		}

		SmartDashboard.putNumber("X ", x);
		SmartDashboard.putNumber("Y ", y);
		SmartDashboard.putNumber("T ", t);

		
			driveTrain.arcadeDrive(-x, t);
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

	void pause( long time ) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Forward;
			targetTime = getTargetTime(time);
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

		double t = (delta < 0) ? targetTurningSpeed : -targetTurningSpeed;

		double d = Math.abs(delta);

		if (d < 10) {
			t = t * (d / 10);
		}

		return t;
	}

	void openDoors() {
	}

	void closeDoors() {
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	@Override
	
	public void pidWrite(double output){
		rotateToAngleRate = output;
	}
}

