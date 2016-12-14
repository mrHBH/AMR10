package parkingRobot.hsamr10;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;

import java.util.Timer;

import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
  
    // #### drive
    double Kp = 0; // 0.
	double Ki = 0;
	double Kd = 0;
	int pi = 0;
	int pd = 0;
	int lastError = 0;
	int differenceError = 1;
	int speed = 0;
	double velocity = 0.0;
	double angularVelocity = 0.0;
	long deltaT = 0;
	double addL = 0;
	double addR = 0;
	double sum = 0;
	// ++ vw control
	double umfang = 2 * Math.PI * 0.028;
	double tempErrorVWLeft = 0;
	double tempErrorVWRight = 0;
	double lastErrorVWLeft = 0;
	double lastErrorVWRight = 0;
	double speedR = 0;
	double speedL = 0;
	double radiusM = 0;
	int wheelDistance = 10; // 10cm
	Timer timer = new Timer();
	private long time = 0;
	private static int mutex = 0;

	
	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor corresponding main module Monitor class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
		this.perception = perception;
        this.navigation = navigation;
        this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	

	
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
		
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
		
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      //testdrive();
		                      break;
		  case VW_CTRL		: update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();		
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity);
	}
	
    private void exec_SETPOSE_ALGO(){
    	//Aufgabe 3.3
	}
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
    
	private void exec_LINECTRL_ALGO(){
		leftMotor.forward();
		rightMotor.forward();
		double Kiline = 0;
		double Kpline = 0.1;
		double Kdline = 1;
		int pi = 0;
		int pd = 0;
		int lastErrorL = 0;
		int differenceErrorL = 1;
		int lastErrorR = 0;
		int differenceErrorR = 1;
		int speed = 0;
		int rightWhite = perception.getLSrwhiteValue();
		int leftWhite = perception.getLSlwhiteValue();
		monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
		if (this.lineSensorLeft == 2 && (this.lineSensorRight == 1)) {
			leftMotor.setPower(1);
			rightMotor.setPower(35);
			monitor.writeControlComment("turn left");
		//	turnBefore = true;
		} else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 1)) { // Rechts
			leftMotor.setPower(35);
			rightMotor.setPower(1);
			monitor.writeControlComment("turn right");
			//turnBefore = true;
		} else if (this.lineSensorLeft == 2 && (this.lineSensorRight == 0)) {
			rightMotor.setPower(45);
			leftMotor.setPower(1);
			monitor.writeControlComment("turn leftKurve");
		//	turnBefore = true;
		} else if (this.lineSensorRight == 2 && (this.lineSensorLeft == 0)) {
			leftMotor.setPower(45);
			rightMotor.setPower(1);
			monitor.writeControlComment("turn rightKurve");
		//	turnBefore = true;
		}

		else if (this.lineSensorLeft == 1 && this.lineSensorRight == 0) {
			int tempValue = perception.getLeftRough();
			perception.getLeftLineSensorValue();
			if (leftWhite - tempValue < 0) {
				differenceErrorL = 0;
			} else {
				if (leftWhite - tempValue > 16) {
					differenceErrorL = 16;
				} else {
					differenceErrorL = leftWhite - tempValue;
				}
			}
			pi = pi + differenceErrorL; // aufintegrieren aller vorherigen
										// Fehler
			pd = differenceErrorL - lastErrorL;
			speed = (int) (((Kpline * differenceErrorL) + (Kiline * pi) + (Kdline * pd)));
			if (speed < 0) {
				speed = 0;
				differenceErrorR = 0;
				lastErrorR = 0;
				rightMotor.setPower(31);// - speed);
				leftMotor.setPower(30);

			} else {
				rightMotor.setPower(35 + speed);// - speed);
				leftMotor.setPower(35);
				monitor.writeControlComment("PIDL1" + speed + "differenceErrorL" + differenceErrorL);
				lastErrorL = differenceErrorL;
			//	turnBefore=true;
			} 
			/*
			 * speed_pid = (int) (0.25 * temp_differenz + 0.25 * (temp_differenz
			 * - last_ldifferenz)); last_ldifferenz = temp_differenz; int pgs =
			 * 45 + speed_pid/4; int ngs = 45 - speed_pid/4; //
			 * monitor.writeControlComment("SpeedPID R0 L1" + pgs+ "end");
			 * rightMotor.setPower(pgs); if (30 - speed_pid >= 0) {
			 * leftMotor.setPower(ngs); } else { leftMotor.setPower(20); }
			 * monitor.writeControlComment("turn left");
			 */
		} else if ((this.lineSensorRight == 1 && this.lineSensorLeft == 0)) {
			// nullmode();
			int tempValue = perception.getRightRough(); // perception.getRightLineSensorValue();

			if (rightWhite - tempValue < 0) {
				differenceErrorL = 0;
			} else {
				if (rightWhite - tempValue > 16) {
					differenceErrorL = 16;
				} else {
					differenceErrorL = rightWhite - tempValue;
				}
			}
			pi = pi + differenceErrorR; // aufintegrieren aller
										// vorherigen//// Fehler
			pd = differenceErrorR - lastErrorR;
			speed = (int) (((Kpline * differenceErrorR) + (Kiline * pi) + (Kdline * pd)));
			if (speed < 0) {
				speed = 0;
				differenceErrorR = 0;
				lastErrorR = 0;

				rightMotor.setPower(30);// - speed);
				leftMotor.setPower(31);
			} else {
				rightMotor.setPower(35);
				leftMotor.setPower(35 + speed);// - speed);
				monitor.writeControlComment("PIDR1L0" + speed + "differenceErrorR" + differenceErrorR);
				monitor.addControlVar("speed" + speed);
				lastErrorR = differenceErrorR;
		//		turnBefore=true;
			} /*
				 * int temp_differenz = rightWhite - tempValue;
				 * 
				 * int speed_pid; speed_pid = (int) (0.25 * temp_differenz +
				 * 0.25 * (temp_differenz - last_rdifferenz)); last_rdifferenz =
				 * temp_differenz; int motorspeed = 45 + (speed_pid/4); int ngs
				 * = 45 - (speed_pid/4);
				 * monitor.writeControlComment("SpeedPID R1 L0" + motorspeed
				 * +"end"); leftMotor.setPower(motorspeed); if (15 - speed_pid
				 * >= 0) { rightMotor.setPower(ngs); } else {
				 * rightMotor.setPower(20); }
				 * 
				 * monitor.writeControlComment("turn right");#+99´
				 * 
				 */
		} else if (this.lineSensorLeft == 2 && this.lineSensorRight == 2) { //
			minischub();
//			turnBefore = false;
		}

		else if (this.lineSensorLeft == 0 && this.lineSensorRight == 0) { //
	//		turnBefore = false;
		//	drive(37, 0);
			leftMotor.setPower(37);
			rightMotor.setPower(37);
		}
	}

	private void minischub() {
		rightMotor.setPower(18); // Um ein abstoppen auf der Linie zu
		// verhindern wird mit minimalem speed
		// weitergefahren,danach sofort gebraked
		leftMotor.setPower(18);

		// brake();
	}
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		

	/**
	 * calculates the left and right angle speed of the both motors with given
	 * velocity and angle velocity of the robot
	 * 
	 * @param v
	 *            velocity of the robot
	 * @param omega
	 *            angle velocity of the robot
	 */

	// Aufgabe 3.2
	private void testdrive() {
		if (mutex == 0) { // count ist global auf 0 initialisiert
			time = System.currentTimeMillis();
			time = time + 30001;
			mutex++;
		}

		if (time != 0) {
			// ######### drive 10cm/s for 15s ###################
			if (System.currentTimeMillis() < (time - 15000)) {
				drive(0.1, 0);
			} else {
				if (System.currentTimeMillis() < (time - 9000)) {
					drive(0, 15);
					resetAdd();
				} else {
					if (System.currentTimeMillis() < (time - 3000)) {
						drive(0.05, 0);
					} else {
						if (System.currentTimeMillis() < (time)) {
							drive(0, 30);
							resetAdd();
						}

						else {
							exec_LINECTRL_ALGO();
						}
					}
				}
			}
		}
	}

	private void drive(double velocity, double winkel) {
		AngleDifferenceMeasurement admL = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement admR = perception.getControlRightEncoder().getEncoderMeasurement();
		addL = addL + admL.getAngleSum();
		addR = addR + admR.getAngleSum();
		leftMotor.forward();
		rightMotor.forward();
		deltaT = admL.getDeltaT();
		// sum = sum + deltaT;
		int wheelDistance = 0; // nochmal unbedingt nachmessen
		double radiusM;
		double velTurn = (((7 / 5D) * (velocity * 100D)) + 20D);
		if (winkel == 0) { // do something to drive straight forward
			radiusM = velocity;//velTurn;
			wheelDistance = 0;
		} else {
			radiusM = velocity / winkel;
			//double radDeg = Math.toRadians(winkel);
			if (radiusM == 0) {

				speedL = (((1 / 5D) * winkel) + 13D);

				leftMotor.setPower((int) -speedL);
				rightMotor.setPower((int) speedL);
				addL = 0;
				addR = 0;
				leftMotor.resetTachoCount();
				rightMotor.resetTachoCount();
				// monitor.writeControlComment("LEftMotorTurn" + (int) speedL +
				// "RightMotorTurn" + (int) -speedL
				// + "anglesumLeft" + addL + "anglesumright" + addR + "end");
			} else {
				wheelDistance = 10;
			}
		}
		double velocityL = ((radiusM - (wheelDistance / 2)) * (velocity / radiusM))-1;
		double velocityR = (radiusM + (wheelDistance / 2)) * (velocity / radiusM);
		if (velocity != 0) {
			// double anglesumL = admL.getAngleSum(); // Returns revolutions
			// after
			// last
			// double angleSumR = admR.getAngleSum();
			tempErrorVWLeft = addL - addR;
			tempErrorVWRight = addR - addL;
			double correctionL = 0;
			double correctionR = 0;
			if (tempErrorVWLeft > 0) {
				correctionL = (0.4 * (tempErrorVWLeft) + 0.8 * (tempErrorVWLeft - lastErrorVWLeft));
				leftMotor.setPower((int) (velocityL - correctionL));
				rightMotor.setPower((int) velocityR);
			} else if (addR - addL > 0) {
				correctionR = (0.4 * (tempErrorVWRight) + 0.8 * (tempErrorVWRight - lastErrorVWRight));
				rightMotor.setPower((int) (velocityR - correctionR));
				leftMotor.setPower((int) velocityL);
			} else if (addL - addR < 1) {
				rightMotor.setPower((int) velocityR);
				leftMotor.setPower((int) velocityL);
			}

			/*
			 * monitor.writeControlComment("LEftMotor" + (int) velocityL +
			 * "RightMotor" + (int) velocityR + "anglesumLeft" + addL +
			 * "anglesumright" + addR + "Rechts " + addR + "Links" + addL +
			 * "correctionL " + correctionL + "correction R" + correctionR +
			 * "end");
			 */
			lastErrorVWLeft = tempErrorVWLeft;
			lastErrorVWRight = tempErrorVWRight;
		}
	}

 private void resetAdd(){
	 addL=0;
	 addR=0;
 }
}

