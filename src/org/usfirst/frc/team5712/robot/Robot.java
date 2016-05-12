package org.usfirst.frc.team5712.robot;

//AUTONOMOUS FUNCTION: Drive through low bar, turn towards low goal, drive towards it, shoot
//ball into low goal

import com.kauailabs.navx.frc.AHRS;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * @author Team 5712 (Mainly Phil, Twigg, Sam, Hunter, and Alex)
 *
 */

public class Robot extends IterativeRobot {

	Joystick driveStick, shootStick;
	
	VictorSP leftFront, leftRear, rightFront, rightRear, shooterR, shooterL, shooterLift;

	Servo servo;
	RobotDrive drive;
	
	SerialPort serial_port;
	
	SendableChooser autonomous;
		
	DoubleSolenoid test1;
	DoubleSolenoid shooter;
	Compressor comp;
	
	AHRS gyro;
	
	Encoder leftDriveEncoder, rightDriveEncoder, shooterEncoder, liftEncoder;

	SendableChooser autoChooser;
	
	//Gyro turning variables
	double angle;
	int lowAutoAngle = 49; //autonomous turning low range 
	int highAutoAngle = 53; //autonomous turning high range
	
	//Encoder tick variables
	int driveTickGoal = 2 * -1000; //tick to distance ratio (in./tick) * distance desired
	int secondDriveGoal = 2 * -600; //tick to distance ratio (in./tick) * distance desired
	double shootTickGoal = 10 * -7.5; //tick to degree ratio (degrees/tick) * angle desired
	int autoLowerShooterArm = 10 * 140; //tick to degree ratio (degrees/tick) * angle desired
	int autoRaiseShooterArm = 10 * 130;
	int highShot = 10 * 45;
	int autoHighShot = 10 * 45;
	int zero = 10 * 0;
	double longShot = 10 * -24.3;
	
	//Camera variables
	int currSession; //current camera
	int sessionFront; //front facing camera
	//int sessionBack; //back facing camera
	int sessionShoot; //camera used for shooting high
	Image frame;
	
	//Boolean condition variables
	boolean start = false;
	boolean autonRan = false;
	boolean invertMotors = false;
	boolean reachedAngle = false;
	boolean shotBall = false;
	boolean punchOut = false;
	long storedTime = 0L;
	int delayState = 0; // The current task of the delay
	
	
	//dio
final int FLASHLIGHT_RELAY = 0;
Relay flashLightRelay = new Relay(FLASHLIGHT_RELAY, Relay.Direction.kForward);
	
	public void robotInit() {
		//Joysticks used for driving
		driveStick = new Joystick(0);
		shootStick = new Joystick(1);
		 
		//Switch on VictorSP to switch between brake and coast mode
		leftFront = new VictorSP(0);
		leftRear = new VictorSP(1); 
		rightFront = new VictorSP(2);
		rightRear = new VictorSP(3);
		shooterL = new VictorSP(4);
		shooterR = new VictorSP(5);
		shooterLift = new VictorSP(6); 

		//Pneumatics 
		test1 = new DoubleSolenoid(2,3);
		shooter = new DoubleSolenoid(0,1);
    	comp = new Compressor(0);
    	comp.setClosedLoopControl(true);

		drive = new RobotDrive(leftFront, leftRear, rightFront, rightRear);

		serial_port = new SerialPort(57600, SerialPort.Port.kMXP);
		byte update_rate_hz = 50;
//hello world
		gyro = new AHRS(SerialPort.Port.kMXP);
		gyro.resetDisplacement();

		//Encoders
		leftDriveEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		rightDriveEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		shooterEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		liftEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
		shooterEncoder.reset();
		liftEncoder.reset();
		
		//Cameras
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		sessionFront = NIVision.IMAQdxOpenCamera("cam0", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		//sessionBack = NIVision.IMAQdxOpenCamera("cam1", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		//sessionShoot = NIVision.IMAQdxOpenCamera("cam2", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		currSession = sessionFront;
		NIVision.IMAQdxConfigureGrab(currSession);
	}

	public void autonomousInit() {
		//======================================================================================
		//									INITIALIZE AUTONOMOUS
		//======================================================================================
		start = true;
		gyro.zeroYaw();
		shooter.set(DoubleSolenoid.Value.kReverse);
		autonRan = false;
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
		shooterEncoder.reset();
		liftEncoder.reset();
	}

	public void autonomousPeriodic() {
		//======================================================================================
		//									BEGIN AUTONOMOUS
		//======================================================================================
		angle = gyro.getYaw();
		//=======================================================
		// Task: Drive straight until designated tick goal
		//=======================================================
		if (leftDriveEncoder.get() > driveTickGoal && reachedAngle == false) {
			//just a test to see how far 100 will get you
			if(shooterEncoder.get() < autoLowerShooterArm){
				shooterLift.set(-0.25);
			} 
			else if (shooterEncoder.get() >= autoLowerShooterArm && reachedAngle == false) {
				shooterLift.set(0);
			
				drive.drive(-0.7, 0.0);
				if (angle > 2)
				{
					rightFront.set(0.8);
					rightRear.set(0.8);
				
				}
				if (angle < -2) 
				{
					leftFront.set(-0.8);
					leftRear.set(-0.8);
				} 
			}

		}
		//=======================================================
		// Task: Turn until desired angle for shooting is reached
		//=======================================================
		if(leftDriveEncoder.get() <= driveTickGoal && reachedAngle == false) {
			if(angle < lowAutoAngle){
				leftFront.set(-0.5 - (40 - angle)/180); 
				leftRear.set(-0.5 - (40 - angle)/180);
				//rightFront.set(-0.4 - (40 - angle)/180);
				//rightRear.set(-0.4 - (40 - angle)/180);
				shooterLift.set(0);
			}
			else if(angle > highAutoAngle){
				leftFront.set(0.4 + (40 - angle)/180); 
				leftRear.set(-0.4 + (40 - angle)/180);
				rightFront.set(0.4 + (40 - angle)/180);
				rightRear.set(0.4 + (40 - angle)/180);
				shooterLift.set(0);
			}
			else if(angle > lowAutoAngle && angle < highAutoAngle){
				leftFront.set(0);
				leftRear.set(0);
				rightFront.set(0);
				rightRear.set(0);
				shooterLift.set(0);
				reachedAngle = true;
				rightDriveEncoder.reset();
				leftDriveEncoder.reset();
			}
		}
		//=======================================================
		// Task: Drive straight until designated tick goal
		//=======================================================
		if (leftDriveEncoder.get() > secondDriveGoal && reachedAngle == true){
			drive.drive(-0.5, 0.0);
			if (angle < lowAutoAngle)
			{
				leftFront.set(-0.6);
				leftRear.set(-0.6);
			
			}
			if (angle > highAutoAngle) 
			{
				rightFront.set(0.6);
				rightRear.set(0.6);
			} 
			if(shooterEncoder.get() > autoRaiseShooterArm) {
				shooterLift.set(0.25);
			}
			if(shooterEncoder.get() <= autoRaiseShooterArm) {
				shooterLift.set(0);
			}
		}
		//=======================================================
		// Task: Shoot boulder into low goal
		//=======================================================		
		else if(leftDriveEncoder.get() < secondDriveGoal && reachedAngle == true && shotBall == false) {
			rightFront.set(0);
			rightRear.set(0);
			leftFront.set(0);
			leftRear.set(0);
			shooterL.set(.5); //set left shooting motor to 1
			shooterR.set(-.5); //set right shooting motor to inverse of left shooting motor
			Timer.delay(.25);
			shooter.set(DoubleSolenoid.Value.kForward);
			Timer.delay(.5);
			shooterL.set(0); //stops the left shooting motor
			shooterR.set(0); //stops the right shooting motor
			shooter.set(DoubleSolenoid.Value.kReverse);
			shotBall = true;
		}
		SmartDashboard.putBoolean("Statement", autonRan);
		SmartDashboard.putNumber("X", gyro.getDisplacementX());
		SmartDashboard.putNumber("Y", gyro.getDisplacementY());
		SmartDashboard.putNumber("Yaw", gyro.getYaw());
		
		SmartDashboard.putNumber("encoder (Left Drive)", leftDriveEncoder.get());
		SmartDashboard.putNumber("encoder (Right Drive)", rightDriveEncoder.get());
		SmartDashboard.putNumber("encoder (Shooter)", shooterEncoder.get()); 
	}
	
	public void teleopInit() {
		//shooterEncoder.reset();
	}
	
	public void teleopPeriodic() {
		//delayState = 0;
		//======================================================================================
		//									DRIVESTICK (DRIVER 1)
		//======================================================================================
		//=======================================================
		// DRIVESTICK AXIS 0 & 1
		// Description: Drive
		//=======================================================
		drive.arcadeDrive(driveStick);
		//=======================================================
		// DRIVESTICK BUTTON 1
		// Description: Hold to shift to high gear
		//=======================================================
		if(driveStick.getRawButton(1) == true){
            test1.set(DoubleSolenoid.Value.kForward);
        }
		else if(driveStick.getRawButton(1) == false){
            test1.set(DoubleSolenoid.Value.kReverse);
		}
		//=======================================================
		// DRIVESTICK BUTTON 3 & 4
		// Description: Reverse motors
		//=======================================================
		if(driveStick.getRawButton(3) == true){
			drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
			drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
			drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
			drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
		}else if(driveStick.getRawButton(4) == true){
			drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
			drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
			drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
			drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
		}
		//=======================================================
		// DRIVESTICK BUTTON 5
		// Description: Turn 145 to 150 degrees
		//=======================================================
		if(driveStick.getRawButton(5) == true) {
			if (gyro.getYaw() > -120) {
				//turn robot until yaw is between -145 and -150
				leftFront.set(-0.4 + (-120 - angle)/180); 
				leftRear.set(0.4 + (-120 - angle)/180);
				rightFront.set(-0.4 + (-120 - angle)/180);
				rightRear.set(-0.4 + (-120 - angle)/180);
			}
			else if(gyro.getYaw() < -123) {
				//turns the robot back if desired angle is passed
				leftFront.set(0.4 - (-120 - angle)/180); 
				leftRear.set(-0.4 - (-120 - angle)/180);
				rightFront.set(0.4 - (-120 - angle)/180);
				rightRear.set(0.4 - (-120 - angle)/180);
			}
			else if(gyro.getYaw() < -120 && gyro.getYaw() > -123) {
				//stop robot when yaw is between -145 and -150
				autonRan = true;
				leftFront.set(0);
				leftRear.set(0);
				rightFront.set(0);
				rightRear.set(0);
			}
		}
		//=======================================================
		// DRIVESTICK BUTTON 6
		// Description: Turn to -135 degrees
		//=======================================================
		if ((driveStick.getRawButton(6) == true)) {
			
			leftFront.set(.5);
			leftRear.set(.5);
			rightFront.set(-.5);
			rightRear.set(-.5);
			if (gyro.getYaw() > -140 && gyro.getYaw() < -130) {
				leftFront.set(0.0);
				leftRear.set(0.0);
				rightFront.set(0.0);
				rightRear.set(0.0);
			}
		}
		//=======================================================
		// DRIVESTICK BUTTON 7 & 8
		// Description: Solenoid out and in
		//=======================================================
		if (driveStick.getRawButton(7) == true) {
			shooter.set(DoubleSolenoid.Value.kForward);
		}
		if (driveStick.getRawButton(8) == true) {
			shooter.set(DoubleSolenoid.Value.kReverse);		
		}
		//=======================================================
		// DRIVESTICK BUTTON 9
		// Description: Switch camera (default is front cam)
		//=======================================================
		/*if (driveStick.getRawButton(9) == true) {
			if (currSession == sessionFront) {
				NIVision.IMAQdxStopAcquisition(currSession);
				currSession = sessionBack;
				NIVision.IMAQdxConfigureGrab(currSession);
			} 
			else if (currSession == sessionBack) {
				NIVision.IMAQdxStopAcquisition(currSession);
				currSession = sessionFront;
				//currSession = sessionShoot;
				NIVision.IMAQdxConfigureGrab(currSession);
			} */
			/*else if (currSession == sessionShoot) {
				NIVision.IMAQdxStopAcquisition(currSession);
				currSession = sessionFront;
				NIVision.IMAQdxConfigureGrab(currSession);
			}*/
		//}
		//=======================================================
		// DRIVESTICK BUTTON 10, 11, & 12
		// Description: No current function
		//=======================================================
		//======================================================================================
		//								END OF DRIVESTICK (DRIVER 1)
		//======================================================================================
		
		//======================================================================================
		//									SHOOTSTICK (DRIVER 2)
		//======================================================================================
		//=======================================================
		// SHOOTSTICK AXIS 1
		// Description: Manually adjust shooter arm
		//=======================================================
		shooterLift.set(-shootStick.getRawAxis(1));
		//=======================================================
		// SHOOTSTICK BUTTON 1
		// Description: Automated shooting
		//=======================================================
		if (shootStick.getRawButton(1) == true) { //when button 1 (the trigger) is pressed...
		drive.arcadeDrive(driveStick); 
		shooterL.set(.5); //set left shooting motor to 1
		shooterR.set(-.5); //set right shooting motor to inverse of left shooting motor
		Timer.delay(.25);
		shooter.set(DoubleSolenoid.Value.kForward); //pushes the ball into the motors to shoot
		Timer.delay(.5);
		shooterL.set(0); //stops the left shooting motor
		shooterR.set(0); //stops the right shooting motor
		shooter.set(DoubleSolenoid.Value.kReverse); //resets the servo angle
	}
//		if (shootStick.getRawButton(1) == true) { //when button 1 (the trigger) is pressed...
//			delayState = 0;
//			drive.arcadeDrive(driveStick); 
//			shooterL.set(.5); //set left shooting motor to 1
//			shooterR.set(-.5); //set right shooting motor to inverse of left shooting motor
//			if (delayState == 0) {
//				if (delay(250)) { //delay of 250 milliseconds
//					shooter.set(DoubleSolenoid.Value.kForward);
//					delayState++;
//				}
//			}
//			else if (delayState == 1) {
//				if (delay(500)) { //delay of 500 milliseconds
//					shooterL.set(0); //stops the left shooting motor
//					shooterR.set(0); //stops the right shooting motor
//					shooter.set(DoubleSolenoid.Value.kReverse);;
//					//would add another delayState++; if we need more operations
//				}
//			}
//		}
		//=======================================================
		// SHOOTSTICK BUTTON 2
		// Description: Shooter intake
		//=======================================================
		if (shootStick.getRawButton(2) == true) {
			shooterL.set(-.5);
			shooterR.set(.5);
		} 
		else if (shootStick.getRawButton(2) == false) {
			shooterL.set(0);
			shooterR.set(0);
		}
		//=======================================================
		// SHOOTSTICK BUTTON 3 & 4
		// Description: Manually adjust shooter arm
		//=======================================================
		/*if (shootStick.getRawButton(3) == true) {
			if (currSession == sessionFront) {
				NIVision.IMAQdxStopAcquisition(currSession);
				currSession = sessionBack;
				NIVision.IMAQdxConfigureGrab(currSession);
			} 
			else if (currSession == sessionBack) {
				NIVision.IMAQdxStopAcquisition(currSession);
				currSession = sessionFront;
				//currSession = sessionShoot;
				NIVision.IMAQdxConfigureGrab(currSession);
			} 
		
		}*/
		
		
		if (shootStick.getRawButton(4) == true) {
			if(shooterEncoder.get() > longShot) { 
			//raise the shooting arm to 75 degrees
				shooterLift.set(0.25);
			}
			else if (shooterEncoder.get() < longShot) {
				shooterLift.set(-0.25);
			}
		}
		//=======================================================
		// SHOOTSTICK BUTTON 5 & 6
		// Description: Adjust shooter arm based on encoders
		//=======================================================
		if (shootStick.getRawButton(5) == true) {
			if(shooterEncoder.get() > shootTickGoal) { 
			//raise the shooting arm to 75 degrees
				shooterLift.set(0.25);
			}
			else if (shooterEncoder.get() < shootTickGoal) {
				shooterLift.set(-0.25);
			}
		}
		if (shootStick.getRawButton(6) == true) {
			if(shooterEncoder.get() > highShot) { 
			//raise the shooting arm to 75 degrees
				shooterLift.set(0.25);
			}
			else if (shooterEncoder.get() < highShot) {
				shooterLift.set(-0.25);
			}
		}
		//=======================================================
		// SHOOTSTICK BUTTON 7, 8, & 9
		// Description: No current function
		//=======================================================
		if (shootStick.getRawButton(7) == true) {
			if(shooterEncoder.get() > zero) { 
			//raise the shooting arm to 75 degrees
				shooterLift.set(0.25);
			}
			else if (shooterEncoder.get() < zero) {
				shooterLift.set(-0.25);
			}
		}
		
		
		if (shootStick.getRawButton(9)) {
			flashLightRelay.set(Relay.Value.kOn);
		} else {
			flashLightRelay.set(Relay.Value.kOff);
		}
		//=======================================================
		//=======================================================
		// SHOOTSTICK BUTTON 10 & 11
		// Description: Solenoid out and in
		//=======================================================
		
		if (shootStick.getRawButton(11) == true) { //when button 1 (the trigger) is pressed...
			drive.arcadeDrive(driveStick);
//			shooterL.set(.7); //set left shooting motor to 1
//			shooterR.set(-.7); //set right shooting motor to inverse of left shooting motor
//			Timer.delay(2);
			shooter.set(DoubleSolenoid.Value.kForward);
			Timer.delay(.20);
//pushes the ball into the motors to shoot
			shooter.set(DoubleSolenoid.Value.kReverse);
//			Timer.delay(1.5);
//			shooterL.set(0); //stops the left shooting motor
//			shooterR.set(0); //stops the right shooting motor
			
		}
//		if (shootStick.getRawButton(11) == true) { //when button 1 (the trigger) is pressed...
		//	delayState = 0;
//			drive.arcadeDrive(driveStick); 
//			shooterL.set(.7); //set left shooting motor to 1
//			shooterR.set(-.7); //set right shooting motor to inverse of left shooting motor
//			if (delayState == 0) {
//				if (delay(2000)) { //delay of 250 milliseconds
//					shooter.set(DoubleSolenoid.Value.kForward);
//					delayState++;
//				}
//			}
//			else if (delayState == 1) {
//				if (delay(1500)) { //delay of 500 milliseconds
//					shooterL.set(0); //stops the left shooting motor
//					shooterR.set(0); //stops the right shooting motor
//					shooter.set(DoubleSolenoid.Value.kReverse);;
//					//would add another delayState++; if we need more operations
//				}
//			}
//		}
		//=======================================================
		// SHOOTSTICK BUTTON 12
		// Description: Shoot at high goal
		//=======================================================
		if (shootStick.getRawButton(12) == true) { //when button 1 (the trigger) is pressed...
			drive.arcadeDrive(driveStick); 
			shooterL.set(.7); //set left shooting motor to 1
			shooterR.set(-.7); //set right shooting motor to inverse of left shooting motor
//			Timer.delay(2);
//			shooter.set(DoubleSolenoid.Value.kForward); //pushes the ball into the motors to shoot
//			Timer.delay(1.5);
//			shooterL.set(0); //stops the left shooting motor
//			shooterR.set(0); //stops the right shooting motor
//			shooter.set(DoubleSolenoid.Value.kReverse); //resets the servo angle
		}
//		if (shootStick.getRawButton(12) == true) { //when button 1 (the trigger) is pressed...
//			delayState = 0;
//			drive.arcadeDrive(driveStick); 
//			shooterL.set(.6); //set left shooting motor to 1
//			shooterR.set(-.6); //set right shooting motor to inverse of left shooting motor
//			if (delayState == 0) {
//				if (delay(2000)) { //delay of 250 milliseconds
//					shooter.set(DoubleSolenoid.Value.kForward);
//					delayState++;
//				}
//			}
//			else if (delayState == 1) {
//				if (delay(1500)) { //delay of 500 milliseconds
//					shooterL.set(0); //stops the left shooting motor
//					shooterR.set(0); //stops the right shooting motor
//					shooter.set(DoubleSolenoid.Value.kReverse);;
//					//would add another delayState++; if we need more operations
//				}
//			}
//		}
		//======================================================================================
		//								END OF SHOOTSTICK (DRIVER 2)
		//======================================================================================
		
		SmartDashboard.putNumber("encoder (Shooter)", shooterEncoder.get());
		NIVision.IMAQdxGrab(currSession, frame, 0);
		CameraServer.getInstance().setImage(frame);
		}
	
	public void testPeriodic() {
	}
	
	public boolean delay(long timeToDelay) {
		// The timeToDelay can be any number type: integer, long, etc.
		
		long currentTime = System.currentTimeMillis();
		boolean finishedDelay = false;
			
		// If storedTime is equal to zero. (Doesn't have a value)
		if(storedTime == 0L) {
		storedTime = currentTime;
					
		} else if(storedTime < currentTime - timeToDelay) {
		finishedDelay = true;
		storedTime = 0L; // Reset the storedTime variables
		// This needs to be done because the next time this method
		// is called, we want the storedTime to update and get the
		// current time.
		}
				
		return finishedDelay;
				
		}

}
