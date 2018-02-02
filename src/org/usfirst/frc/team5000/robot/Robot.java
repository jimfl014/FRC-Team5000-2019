package org.usfirst.frc.team5000.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	WPI_TalonSRX driveCimLF, driveCimLR, driveCimRF, driveCimRR;
	Joystick driveJoystick;
	HHJoystickButtons driveJoystickButtons;
	DifferentialDrive driveTrain;
	
	DoubleSolenoid actuator1, actuator2;
	
	static final int forwardChannel1 = 0;
	static final int reverseChannel1 = 1;
	static final int forwardChannel2 = 2;
	static final int reverseChannel2 = 3;
	static final int actuator1FWD = 2;
	static final int actuator1REV = 3;
	//static final int actuator2FWD = 4;
	//static final int actuator2REV = 5;
			
	AHRS ahrs;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		driveCimLF = new WPI_TalonSRX(0);
		driveCimLR = new WPI_TalonSRX(1);
		driveCimRF = new WPI_TalonSRX(2);
		driveCimRR = new WPI_TalonSRX(3);
		
		SpeedControllerGroup right = new SpeedControllerGroup(driveCimRF,driveCimRR);
		SpeedControllerGroup left = new SpeedControllerGroup(driveCimLF,driveCimLR);
		
		driveTrain = new DifferentialDrive(right,left);
		driveJoystick = new Joystick(0);
		driveJoystickButtons = new HHJoystickButtons(driveJoystick, 10);
	
		actuator1 = new DoubleSolenoid(forwardChannel1,reverseChannel1);
		actuator2 = new DoubleSolenoid(forwardChannel2,reverseChannel2);
		
		// ahrs = new AHRS(SerialPort.Port.kMXP.kUSB);
		ahrs = new AHRS(Port.kUSB);
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
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
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
		
		driveTrain.arcadeDrive(driveJoystick.getY(), driveJoystick.getZ());
		
		openCloseClaw();
		
		SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
	}
	
	void openCloseClaw() {
		
		if (driveJoystickButtons.isPressed(actuator1FWD)) {
			actuator1.set(DoubleSolenoid.Value.kReverse);
			// Timer.delay(0.1);
			actuator2.set(DoubleSolenoid.Value.kForward);
			Timer.delay(0.5);
			actuator2.set(DoubleSolenoid.Value.kReverse);
		} else if (driveJoystickButtons.isPressed(actuator1REV)) {
				actuator1.set(DoubleSolenoid.Value.kForward);
			}
		//actuator1.set(DoubleSolenoid.Value.kOff);
	}
	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

