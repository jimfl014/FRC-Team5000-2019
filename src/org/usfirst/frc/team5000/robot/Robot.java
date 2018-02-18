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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CameraServer;

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
		InitialOpen, Close, SecondOpen, Push, LiftAtTop
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

	static final String Left = "Left";
	static final String Center = "Center";
	static final String Right = "Right";

	static final String LeftColorRightAuto = "LCR";
	static final String LeftColorLeftAuto = "LCL";
	static final String CenterColorLeftAuto = "CCL";
	static final String CenterColorRightAuto = "CCR";
	static final String RightColorLeftAuto = "RCL";
	static final String RightColorRightAuto = "RCR";

	static final double Kp = 0.03;

	static final int forwardChannel1 = 0; //this is open
	static final int reverseChannel1 = 3; //this is close
	static final int armsopen = 1;
	static final int armsclose = 2;
	static final int forwardChannel2 = 2; //this is out
	static final int reverseChannel2 = 1; //this is in
	static final long delaybeforepush = 000;
	static final long delaybeforeretract = 2500;

	static final long delaybeforedrop = 1000;
	static final int droplift = 4;
	static final int droplift2 = 5;

	static final int winchup = 7;
	static final int winchup2 = 10;
	static final int winchdown = 8;
	static final int winchdown2 = 9;

	static final int resetrotate = 4;
	//static final int rotatezero = 5;
	//static final int rotateninety = 6;
	//static final int rotateoneeighty = 7;
	//static final int rotatetwoseventy = 8;

	static final int actuator1FWD = 2;
	static final int actuator1REV = 3;
	//static final int actuator2FWD = 4;
	//static final int actuator2REV = 5;

	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

	static final double kToleranceDegrees = 2.0f;
	static final double kCollisionThreshold_DeltaG = 0.5f;

	static final int LIFT_UP_BUTTON = 3;
	static final double FORWARD_LIFT_SPEED = -0.7;//change for actual robot
	static final double REVERSE_LIFT_SPEED = 0.6;//change for actual robot

	static final double ForwardWinchSpeed = -0.7;
	static final double ReverseWinchSpeed = 0.6;

	static final double drivestopexception = 6;
	static final double drivestopexception2 = 11;

	static final double CurrentLimit = 25;

	static final double AUTO_SPEED = 0.3;

    static final String TIME_TO_BASELINE_WIDGET = "Time to drive to baseline ";
    static final String TIME_TO_DRIVE_NEXT_TO_SWITCH_WIDGET = "Time to drive next to switch ";
    static final String TIME_TO_DRIVE_TO_SIDE_OF_SWITCH_WIDGET = "Time to drive to side of switch ";
    static final String TIME_TO_DRIVE_TO_FRONT_OF_SWITCH_WIDGET = "Time to drive to front of switch ";
    static final String TIME_TO_DRIVE_TO_CORNER_OF_SWITCH_WIDGET = "Time to drive to corner of switch ";
    static final String TIME_TO_DRIVE_NEXT_TO_SWITCH_FROM_CORNER_WIDGET = "Time to drive next to switch from corner ";
    static final String TIME_TO_DRIVE_TO_SIDE_OF_SWITCH_FROM_CENTER_WIDGET = "Time to drive to side of switch from center ";

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
	double baseDriveLevel = 0.4;
	double baseTwistLevel = 0.6;
	long driveToBaseLineTime = 5000;
	long driveNextToSwitch = 2000;
	long driveToSideOfSwitch = 1000;
	long driveToFrontOfSwitch = 1000;
	long driveToCornerOfSwitch = 1000;
	long driveNextToSwitchFromCorner = 1000;
	long driveToSideOfSwitchFromCenter = 1000;
	long pauseBetweenLiftAndPush = 3000;
	boolean drivejoystickenabled = false;
	
	long autoDefaultDriveTime = 1000;

	CameraServer cameraServer;

	DigitalInput irSensorLeft, irSensorRight;
	boolean leftIRSensor, rightIRSensor;

	WPI_TalonSRX driveCimLF, driveCimLR, driveCimRF, driveCimRR;
	Joystick driveJoystick, buttonsJoystick;
	HHJoystickButtons buttonsJoystickButtons;
	DifferentialDrive driveTrain;

	DoubleSolenoid arms, pusher;
	Spark lift, winch;
	MotorState liftState = MotorState.Stopped;
	MotorState winchState = MotorState.Stopped;

	PowerDistributionPanel pdp;

	long firstdelay = 0;
	long seconddelay = 0;
	long thirddelay = 0;

	AHRS ahrs;

    boolean collisionDetected = false;
	double last_world_linear_accel_x;
	double last_world_linear_accel_y;
    
	PIDController turnController;
	double rotateToAngleRate;
	boolean rotateToAngle = false;

	ClawState clawState;

	String autoMode;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Left", Left);
		chooser.addObject("Center", Center);
		chooser.addObject("Right", Right);
		SmartDashboard.putData("Auto choices", chooser);

		SmartDashboard.putString("Auto Step ", AutoStep.Start.toString());

		//driveCimLF = new WPI_TalonSRX(0);
		driveCimLR = new WPI_TalonSRX(1);
		//driveCimRF = new WPI_TalonSRX(2);
		driveCimRR = new WPI_TalonSRX(3);

		driveCimLR.setInverted(true);
		driveCimRR.setInverted(true);

		SpeedControllerGroup right = new SpeedControllerGroup(/*driveCimRF,*/driveCimRR);
		SpeedControllerGroup left = new SpeedControllerGroup(/*driveCimLF,*/driveCimLR);

		driveTrain = new DifferentialDrive(left,right);
		driveJoystick = new Joystick(0);		
		buttonsJoystick = new Joystick(1);
		buttonsJoystickButtons = new HHJoystickButtons(buttonsJoystick, 10);

		arms = new DoubleSolenoid(forwardChannel1, reverseChannel1);
		pusher = new DoubleSolenoid(forwardChannel2, reverseChannel2);
		
		ahrs = new AHRS(SPI.Port.kMXP);
		//ahrs = new AHRS(I2C.Port.kOnboard);

		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0,  1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		lift = new Spark(0);
		winch = new Spark(1);

		cameraServer = CameraServer.getInstance();
		cameraServer.startAutomaticCapture("cam0", 0);

		pdp = new PowerDistributionPanel();
		
		SmartDashboard.putString(TIME_TO_BASELINE_WIDGET, Long.toString(driveToBaseLineTime));
		SmartDashboard.putString(TIME_TO_DRIVE_NEXT_TO_SWITCH_WIDGET, Long.toString(driveNextToSwitch));
		SmartDashboard.putString(TIME_TO_DRIVE_TO_SIDE_OF_SWITCH_WIDGET, Long.toString(driveToSideOfSwitch));
		SmartDashboard.putString(TIME_TO_DRIVE_TO_FRONT_OF_SWITCH_WIDGET, Long.toString(driveToFrontOfSwitch));
		SmartDashboard.putString(TIME_TO_DRIVE_TO_CORNER_OF_SWITCH_WIDGET, Long.toString(driveToCornerOfSwitch));
		SmartDashboard.putString(TIME_TO_DRIVE_NEXT_TO_SWITCH_FROM_CORNER_WIDGET, Long.toString(driveNextToSwitchFromCorner));
		SmartDashboard.putString(TIME_TO_DRIVE_TO_SIDE_OF_SWITCH_FROM_CENTER_WIDGET, Long.toString(driveToSideOfSwitchFromCenter));
		
		SmartDashboard.putNumber("Base Drive Level ", baseDriveLevel);
		SmartDashboard.putNumber("Base Twist Level ", baseTwistLevel);
		SmartDashboard.putBoolean("Collision Detected ", collisionDetected);
	}

	/** Need to add teleopInit JF
	 * 
	 */
	@Override
	public void teleopInit() {
		turnController.disable();
		rotateToAngle = false;
		liftState = MotorState.Stopped;
		winchState = MotorState.Stopped;
		clawState = ClawState.InitialOpen;
		drivejoystickenabled = true;
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		buttonsJoystickButtons.updateState();

		openCloseClaw();
		liftPeriodic();
		winchPeriodic();

		SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
		SmartDashboard.putNumber("IMU_Angle", ahrs.getAngle());
		SmartDashboard.putBoolean("PID enabled", turnController.isEnabled());
		SmartDashboard.putNumber("PID setPoint", turnController.getSetpoint());
		SmartDashboard.putNumber("PID delta", turnController.getDeltaSetpoint());
		SmartDashboard.putNumber("PID rotateToAngleRate", rotateToAngleRate);

		/*if(buttonsJoystickButtons.isPressed(resetrotate)){
			ahrs.reset();
		} //else if (buttonsJoystickButtons.isPressed(rotatezero)){
			//turnController.setSetpoint(0.0f);
			//rotateToAngle = true;
		} else if (buttonsJoystickButtons.isPressed(rotateninety)){
			turnController.setSetpoint(90.0f);
			rotateToAngle = true;
		} else if (buttonsJoystickButtons.isPressed(rotateoneeighty)){
			turnController.setSetpoint(179.9f);
			rotateToAngle = true;
		} else if (buttonsJoystickButtons.isPressed(rotatetwoseventy)){
			turnController.setSetpoint(-90.0f);
			rotateToAngle = true;
		} else if (rotateToAngle && turnController.onTarget()) {
			rotateToAngle = false;
		}*/

		double currentRotationRate;

		if (rotateToAngle && false){
			turnController.enable();
			currentRotationRate = rotateToAngleRate;
		} else {
			turnController.disable();
			currentRotationRate = driveJoystick.getTwist();
		}

		/*try {
			driveTrain.arcadeDrive(driveJoystick.getY(), currentRotationRate  * Math.abs(currentRotationRate));
		} catch(RuntimeException ex) {
			DriverStation.reportError("Error communicating with drive system: "+ ex.getMessage(), true);
		}*/
		double y = driveJoystick.getY();
		double t = currentRotationRate * Math.abs(currentRotationRate);
		SmartDashboard.putNumber("Y ", y);
		SmartDashboard.putNumber("T ", t);

		driveTrain.arcadeDrive(y, t);
	}

	void openCloseClaw() {

		SmartDashboard.putString("ClawState", clawState.toString());
		switch (clawState) {
		case InitialOpen:
			arms.set(DoubleSolenoid.Value.kForward); //arms open
			pusher.set(DoubleSolenoid.Value.kForward); //pusher retracted
			if (buttonsJoystickButtons.isPressed(armsclose)) {
				clawState = ClawState.Close;
			}
			break;
		case Close:
			arms.set(DoubleSolenoid.Value.kReverse); //arms close
			if (buttonsJoystickButtons.isPressed(armsopen)) {
				clawState = ClawState.SecondOpen;
				firstdelay = System.currentTimeMillis() + delaybeforepush;
			}
			break;
		case SecondOpen:
			arms.set(DoubleSolenoid.Value.kForward); // arms open
			if(firstdelay <= System.currentTimeMillis()){
				clawState = ClawState.Push;
				seconddelay = System.currentTimeMillis() + delaybeforeretract;
			}
			break;
		case Push:
			pusher.set(DoubleSolenoid.Value.kReverse); //pusher out
			if(seconddelay <= System.currentTimeMillis()){
				clawState = ClawState.InitialOpen;
			}
			break;
		case LiftAtTop:
			firstdelay = System.currentTimeMillis() + delaybeforepush;
			clawState = ClawState.SecondOpen;
			break;
		} 
	}

	void liftPeriodic() {
		String liftStatus = null;
		SmartDashboard.putString("LiftState", liftState.toString());

		double Current = pdp.getCurrent(14);//change # for actual robot
		SmartDashboard.putNumber("Lift Current", Current);

		if (buttonsJoystickButtons.isPressed(LIFT_UP_BUTTON)) {
			//if(liftState == MotorState.Stopped){
			liftState = MotorState.Forward;
			//}
			//else if (liftState == MotorState.Forward){
			//liftState = MotorState.Stopped;
			//}
		}
		if (buttonsJoystickButtons.isReleased(LIFT_UP_BUTTON)){
			liftState = MotorState.Stopped;					   
		}
		if (Current >= CurrentLimit){ //change # for actual robot
			liftState = MotorState.Stopped;
			clawState = ClawState.LiftAtTop;
		}

		thirddelay = System.currentTimeMillis() + delaybeforedrop;

		if (/*buttonsJoystickButtons.isPressed(droplift) || */buttonsJoystickButtons.isPressed(droplift2)){
			liftState = MotorState.Reverse;
		}
		if (/*buttonsJoystickButtons.isReleased(droplift) || */buttonsJoystickButtons.isReleased(droplift2)){
			liftState = MotorState.Stopped;
		}

		if (thirddelay <= System.currentTimeMillis()){
			liftState = MotorState.Stopped;
		}

		if (liftState == MotorState.Forward) {
			lift.set(FORWARD_LIFT_SPEED);
			liftStatus = "Up";
			//drivejoystickenabled = false;
		} else if (liftState == MotorState.Reverse){
			lift.set(REVERSE_LIFT_SPEED);
			liftStatus = "Drop";
		}
		else {
			lift.stopMotor();
			liftStatus = "Stopped";
			//drivejoystickenabled = true;
		}

		SmartDashboard.putString("Lift Status", liftStatus);
	}

	void winchPeriodic() {
		String winchStatus = null;
		SmartDashboard.putString("Winch State ", winchState.toString());

		double Current = pdp.getCurrent(14);//change # for actual robot
		SmartDashboard.putNumber("Winch Current ", Current);

		if (buttonsJoystickButtons.isPressed(winchup) || buttonsJoystickButtons.isPressed(winchup2)){
			winchState = MotorState.Forward;
		}

		if (buttonsJoystickButtons.isReleased(winchup) || buttonsJoystickButtons.isReleased(winchup2)){
			winchState = MotorState.Stopped;
		}

		if (buttonsJoystickButtons.isPressed(winchdown) || buttonsJoystickButtons.isPressed(winchdown2)){
			winchState = MotorState.Reverse;
		}

		if (buttonsJoystickButtons.isReleased(winchdown) || buttonsJoystickButtons.isReleased(winchdown2)){
			winchState = MotorState.Stopped;
		}

		if(winchState == MotorState.Forward){
			winch.set(ForwardWinchSpeed);
			winchStatus = "Up";
		} 
		else if (winchState == MotorState.Reverse){
			winch.set(ReverseWinchSpeed);
			winchStatus = "Down";
		}
		else{
			winch.stopMotor();
			winchStatus = "Stopped";
		}
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
		autoSelected = chooser.getSelected();
		if (autoSelected == null){
			autoSelected = Left;
			System.out.println("Auto selected was null. Reset to Left.");
		}
		System.out.println("Auto selected: " + autoSelected);

		ahrs.reset();

		autoStep = AutoStep.Start;
		SmartDashboard.putString("Auto Step ", autoStep.toString());

		initializeForNextStep();


		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(gameData.length() > 0){
			if(gameData.charAt(0) == 'L'){
				switch(autoSelected){
				case Left:
					autoMode = LeftColorLeftAuto;
					break;
				case Center:
					autoMode = CenterColorLeftAuto;
					break;
				case Right:
					autoMode = RightColorLeftAuto;
					break;
				}
			}else{
				switch(autoSelected){
				case Left:
					autoMode = LeftColorRightAuto;
					break;
				case Center:
					autoMode = CenterColorRightAuto;
					break;
				case Right:
					autoMode = RightColorRightAuto;
					break;
				}
			}
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		currentAngle = ahrs.getAngle();

		SmartDashboard.putString("IMU_Angle", String.format("%.2f", currentAngle));

		angularDistance = getAngularDistanceFromTarget(currentAngle, targetAngle);

		SmartDashboard.putNumber("Target Angle ", targetAngle);
		SmartDashboard.putNumber("D ", angularDistance);

                detectCollision();

                SmartDashboard.putBoolean( "Collision Detected ", collisionDetected);
                
		leftIRSensor = irSensorLeft != null ? irSensorLeft.get() : false;
		rightIRSensor = irSensorRight != null ? irSensorRight.get() : false;

		SmartDashboard.putString("Left IR  ", Boolean.toString(leftIRSensor));
		SmartDashboard.putString("Right IR ", Boolean.toString(rightIRSensor));

		boolean incrementStep = false;

		if (targetTime == 0 && targetTurningSpeed == 0) {
			incrementStep = true;
		} else if (0 < targetTime && targetTime <= System.currentTimeMillis()) {
			incrementStep = true;
		} else if (targetTurningSpeed > 0 && turnController.onTarget()) {
			incrementStep = true;
		} else if (targetSpeed > 0 && collisionDetected) {
			incrementStep = true;
		} else if (watchForReflectiveStrips && leftIRSensor && rightIRSensor) {
			incrementStep = true;
		}

		if (incrementStep) {
			initializeForNextStep();
			autoStep = autoStep.next();

			SmartDashboard.putString("Auto Step ", autoStep.toString());
		}

		switch (autoMode) {
		case LeftColorRightAuto:
			leftcolorright();
			break;

		default:
		case LeftColorLeftAuto:
			leftcolorleft();
			break;

		case CenterColorLeftAuto:
			centercolorleft();
			break;

		case CenterColorRightAuto:
			centercolorright();
			break;

		case RightColorLeftAuto:
			leftcolorright();
			break;

		case RightColorRightAuto:
			rightcolorright();
			break;
		}

		doStep();
	}

	void leftcolorright() {

		switch (autoStep) {
		case Start:
			break;

		case Step1:
                    long driveTime = getDashboardValue( TIME_TO_BASELINE_WIDGET, driveToBaseLineTime );

			driveForward(AUTO_SPEED, driveTime);
			break;

		case Stop:
		default:
			stop();
			break;
		}
	}

	void leftcolorleft() {
		long driveTime = 0;

		switch (autoStep) {
		case Start:
			break;

		case Step1:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_NEXT_TO_SWITCH_WIDGET, driveNextToSwitch );

			driveForward(AUTO_SPEED, driveTime );
			break;

		case Step2:
			turnTo(0.10, 90);
//			watchForReflectiveStrips = true;
			break;

		case Step3:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_TO_SIDE_OF_SWITCH_WIDGET, driveToSideOfSwitch );

			driveForward(AUTO_SPEED, driveTime );
			break;

		case Step4: 
			liftcube();
			pause(pauseBetweenLiftAndPush);
			break;

		case Step5:
                    driveTime = 1000;

			driveForward(AUTO_SPEED, driveTime );
			break;
			
		case Step6:
			shootcube();
			break;

		case Stop:
		default:
			stop();
			break;
		}
	}

	void centercolorleft() {
		long driveTime = 0;

		switch (autoStep) {
		case Start:
			break;

		case Step1:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_TO_FRONT_OF_SWITCH_WIDGET, driveToFrontOfSwitch );

			driveForward(AUTO_SPEED, driveTime);
			break;

		case Step2:
			turnTo(0.10, -90);
			break;

		case Step3:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_TO_CORNER_OF_SWITCH_WIDGET, driveToCornerOfSwitch );

			driveForward(AUTO_SPEED, driveTime);
			break;

		case Step4:
			turnTo(0.10, 0);
			break;

		case Step5:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_NEXT_TO_SWITCH_FROM_CORNER_WIDGET, driveNextToSwitchFromCorner );
                    
			driveForward(AUTO_SPEED, driveTime);
			break;

		case Step6:
			turnTo(0.10, 90);
			break;

		case Step7:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_TO_SIDE_OF_SWITCH_FROM_CENTER_WIDGET, driveToSideOfSwitchFromCenter );
                    
			driveForward(AUTO_SPEED, driveTime);
			break;

		case Step8:
			liftcube();
			pause(pauseBetweenLiftAndPush);
			break;
			
		case Step9:
                    driveTime = 1000;

			driveForward(AUTO_SPEED, driveTime );
			break;
			
		case Step10:
			shootcube();
			break;
			
		case Stop:
			default:
				stop();
			break;
		}
	}

	void centercolorright() {
		long driveTime = 0;

		switch (autoStep) {
		case Start:
			break;

		case Step1:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_TO_FRONT_OF_SWITCH_WIDGET, driveToFrontOfSwitch );

			driveForward(AUTO_SPEED, driveTime);
			break;

		case Step2:
			turnTo(0.10, 90);
			break;

		case Step3:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_TO_CORNER_OF_SWITCH_WIDGET, driveToCornerOfSwitch );

			driveForward(AUTO_SPEED, driveTime);
			break;

		case Step4:
			turnTo(0.10, 0);
			break;

		case Step5:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_NEXT_TO_SWITCH_FROM_CORNER_WIDGET, driveNextToSwitchFromCorner );

			driveForward(AUTO_SPEED, driveTime);
			break;

		case Step6:
			turnTo(0.10, -90);
			break;

		case Step7:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_TO_SIDE_OF_SWITCH_FROM_CENTER_WIDGET, driveToSideOfSwitchFromCenter );

			driveForward(AUTO_SPEED, driveTime);
			break;

		case Step8:
			liftcube();
			pause(pauseBetweenLiftAndPush);
			break;
			
		case Step9:
                    driveTime = 1000;

			driveForward(AUTO_SPEED, driveTime );
			break;
			
		case Step10:
			shootcube();
			break;
			
		case Stop:
			default:
				stop();
			break;
		}
	}

	void rightcolorright() {
		long driveTime = 0;

		switch (autoStep) {
		case Start:
			break;

		case Step1:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_NEXT_TO_SWITCH_WIDGET, driveNextToSwitch );

			driveForward(AUTO_SPEED, driveTime );
			break;

		case Step2:
			turnTo(0.10, -90);
//			watchForReflectiveStrips = true;
			break;

		case Step3:
                    driveTime = getDashboardValue( TIME_TO_DRIVE_TO_SIDE_OF_SWITCH_WIDGET, driveToSideOfSwitch );

			driveForward(AUTO_SPEED, driveTime );
			break;

		case Step4: 
			liftcube();
			pause(pauseBetweenLiftAndPush);
			break;

		case Step5:
                    driveTime = 1000;

			driveForward(AUTO_SPEED, driveTime );
			break;
			
		case Step6:
			shootcube();
			break;

		case Stop:
		default:
			stop();
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
//			t = rotateToAngleRate;
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
			t = rotateToAngleRate;
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

		driveTrain.arcadeDrive(-y, t);
	}

	void initializeForNextStep() {
		targetTime = 0;
		targetSpeed = 0;
		targetAngle = currentAngle;
		targetTurningSpeed = 0;
		watchForReflectiveStrips = false;
		turnController.disable();
                collisionDetected = false;
		initAutoStep = true;
	}

	void driveForward(double speed, long time) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Forward;
			targetSpeed = speed;
			targetTime = getTargetTime(time);
			targetAngle = currentAngle;
//			turnController.setSetpoint(targetAngle);
//			turnController.enable();		
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

	void turnTo(double speed, double angle) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Turn;
			targetTurningSpeed = speed;
			targetAngle = angle;
			turnController.setSetpoint(targetAngle);
			turnController.enable();
		}
	}

	void turnLeft(double speed, double angle) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Turn;
			targetTurningSpeed = speed;
			targetAngle = getTargetAngle(currentAngle, -angle);
			turnController.setSetpoint(targetAngle);
			turnController.enable();
		}
	}

	void turnRight(double speed, double angle) {
		if (initAutoStep) {
			initAutoStep = false;
			driveState = DriveState.Turn;
			targetTurningSpeed = speed;
			targetAngle = getTargetAngle(currentAngle, angle);
			turnController.setSetpoint(targetAngle);
			turnController.enable();
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

	void liftcube() {
		lift.set(FORWARD_LIFT_SPEED);
	}

	void shootcube(){
		lift.stopMotor();
		arms.set(DoubleSolenoid.Value.kForward);
		pusher.set(DoubleSolenoid.Value.kReverse);
	}

	void dropcube() {
	}

    long getDashboardValue( final String widget, long defaultValue ) {

        long value;

        try {
            value = Long.parseLong(SmartDashboard.getString(widget, "bad"));
        } catch (NumberFormatException e){
            value =  defaultValue;
        }

        return value;
    }

    void detectCollision() {

        double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
             ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
            collisionDetected = true;
        }
    }

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	@Override

	public void pidWrite(double output){
		rotateToAngleRate = output;
	}
}
