package parkingRobot.hsamr10;

import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;

import java.util.Timer;

import lejos.geom.Point;
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {

	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel
	 * which measures the wheels angle difference between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft = null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot
	 * wheel which measures the wheels angle difference between actual an last
	 * request
	 */
	IPerception.EncoderSensor encoderRight = null;

	/**
	 * reference to data class for measurement of the left wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft = null;
	/**
	 * reference to data class for measurement of the right wheels angle
	 * difference between actual an last request and the corresponding time
	 * difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight = null;

	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorRight = 0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on
	 * line border or gray underground, 2 - on line
	 */
	int lineSensorLeft = 0;

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
	EncoderSensor controlRightEncoder = null;
	EncoderSensor controlLeftEncoder = null;
	int lastTime = 0; // #
	double currentDistance = 0.0; // #
	double Distance = 0.0; // #
	// ####### Linefollower ###################################################
	int lastErrorL = 0;
	int differenceErrorL = 1;
	int integralLine = 0;
	float Kpline = 1.4f;
	float KiLine = 0f;
	float Kdline = 0.9f;
	int pdline = 0;
	int ksline = 0;
	int offset = 30;
	// #### drive ###########################################################
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
	// ########### vw control ########################
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
	double integral = 0;
	double difference = 0;
	double fixAddL = 0;
	double fixAddR = 0;
	int mutexTurn = 0;
	int mutexStraight = 0;
	// #### Set Pose #####
	Pose currentPositionDegrees = new Pose();
	boolean desiredHeading = false;
	float yStart = currentPosition.getY();
	float xStart = currentPosition.getX();
	double phi = 0;
	double calculatedPositionBefore = 0;
	boolean finalDestination = false;
	boolean finalLocation = false;
	int mutexGetHeading = 0;
	int mutexSetPose = 0;
	// ##### drive #####
	double pidL = 0;
	double pidR = 0;
	double integralL = 0;
	double integralR = 0;
	double differenceL = 0;
	double differenceR = 0;
	double lastErrorR;
	double lastErrorLinks;
	double powerL = 0;
	double powerR = 0;
	double tempSpeedLeft = 0;
	double tempSpeedRight = 0;
	// ##### ParkIN######
	double x = -1;
	double w = 0;
	double velo = 0;
	double x_back = 0;
	int mutex_out = 0;
	public boolean parking = false;
	// Testsequenz
	double radsummeL = 0;
	boolean turn90 = false;
	int turn90_mutex = 0;
	boolean driven10 = false;
	boolean turn15 = false;
	boolean driven5 = false;
	boolean turn30 = false;
	float call = 0;
	public boolean succesfullyParkedOut = false;
	public boolean succesfullyParkedIn = false;
	
	

	/**
	 * provides the reference transfer so that the class knows its corresponding
	 * navigation object (to obtain the current position of the car from) and
	 * starts the control thread.
	 * 
	 * @param perception
	 *            corresponding main module Perception class object
	 * @param navigation
	 *            corresponding main module Navigation class object
	 * @param monitor
	 *            corresponding main module Monitor class object
	 * @param leftMotor
	 *            corresponding NXTMotor object
	 * @param rightMotor
	 *            corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor,
			IMonitor monitor) {
		this.perception = perception;
		this.navigation = navigation;
		this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.currentCTRLMODE = ControlMode.INACTIVE;

		this.encoderLeft = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();

		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");

		this.ctrlThread = new ControlThread(this);

		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to
									// terminate in order for the user program
									// to terminate
		ctrlThread.start();
	}

	// Inputs

	/**
	 * set velocity
	 * 
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * 
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}

	/**
	 * set destination
	 * 
	 * @see parkingRobot.IControl#setDestination(double heading, double x,
	 *      double y)
	 */
	public void setDestination(double heading, double x, double y) {
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}

	/**
	 * sets current pose
	 * 
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
	 * Function to do Linefollowing backwords to avoid collisions must be called
	 * from Guidance !!!!!
	 */
	public void setBackwords(boolean back) {
		if (back == true) {
			this.offset = -25;
		} else
			this.offset = 25;
	}

	/**
	 * set start time
	 */
	public void setStartTime(int startTime) {
		this.lastTime = startTime;
	}

	/**
	 * selection of control-mode
	 * 
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO() {

		switch (currentCTRLMODE) {
		case LINE_CTRL:
			// setBackwords(true);
			update_LINECTRL_Parameter();
		//	setBackwords(false);
		// exec_LINECTRL_ALGO();
			// testdrive();
			update_SETPOSE_Parameter();
			//ParkOut();// 
		//	exec_PARKCTRL_ALGO() ;
			
			// ParkNow();
			// exec_SETPOSE_ALGO();
			// ParkIn();
			//testSequenz1();// driveNEW(0.1, -35);
			//
	ParkBackwords();
			break;
		case VW_CTRL:
			update_VWCTRL_Parameter();
			exec_VWCTRL_ALGO();
			break;
		case SETPOSE:
			update_SETPOSE_Parameter();
			exec_SETPOSE_ALGO();
			break;
		case PARK_CTRL:
			update_PARKCTRL_Parameter();
			exec_PARKCTRL_ALGO();
			break;
		case INACTIVE:
			exec_INACTIVE();
			break;
		}

	}

	// Private methods

	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter() {
		setPose(navigation.getPose());
	}

	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter() {
		setPose(navigation.getPose());
		currentPositionDegrees.setLocation(currentPosition.getLocation());
		currentPositionDegrees.setHeading((float) (currentPosition.getHeading() * (180 / Math.PI)));
	}

	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter() {

	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter() {
		this.lineSensorRight = perception.getRightLineSensor();
		this.lineSensorLeft = perception.getLeftLineSensor();
	}

	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade
	 * during VW Control Mode optionally one of them could be set to zero for
	 * simple test.
	 */
	private void exec_VWCTRL_ALGO() {
		this.drive(this.velocity, this.angularVelocity);
	}

	/**
	 * This is a function which turns the robot at a current position. The angle
	 * it is turned is calculated in Degrees with the help of function
	 * relativeBearing, which is returning an angle in degrees. This is the
	 * reason why I had to implement a new Pose, where the heading is given in
	 * degrees
	 */
	private void turnToLocation() {
		if (desiredHeading == false) {
			float degree = currentPositionDegrees.relativeBearing(destination.getLocation());
			if (degree < (-5f)) {
				drive(0, -20);

			} else {
				if (degree > 5) {
					drive(0, 20);
				} else {
					stop();
					desiredHeading = true;
					monitor.writeControlComment("TurnLocation == true");
					return;

				}
				monitor.writeControlComment("TurnToLocation" + degree + desiredHeading);
			}
		}
		/*
		 * monitor.writeControlComment("TurnToLocation" +
		 * currentPositionDegrees.relativeBearing(destination.getLocation()) +
		 * "X " + currentPosition.getX() + "Y" + currentPosition.getY());
		 */
	}

	/**
	 * This function will turn the robot after he received his final position
	 * Therefore the remaining angle ( to turnDegrees ) is calculated. We have
	 * to pay attention to overflows after calculation
	 */
	private void turnDefinedAngle() {
		if (!finalDestination) {
			update_SETPOSE_Parameter();
			double turnDegrees;
			turnDegrees = ((180 / Math.PI) * (destination.getHeading())
					- ((180 / Math.PI) * currentPosition.getHeading()));
			// ######## to avoid an overflow ########
			if (turnDegrees < -180) {
				turnDegrees = turnDegrees + 360;
			} else if (turnDegrees > 180) {
				turnDegrees = turnDegrees - 360;
			}
			// ######## turn until MAth.abs(angle) =1 #####
			if (turnDegrees > 8) {
				drive(0, 30);
			} else if (turnDegrees < -8) {
				drive(0, -30);
			} else {
				// ### break the while because we have reached the desired
				// Heading
				stop();
				finalDestination = true;
			}
			monitor.writeControlComment(
					"TurnDefinedAngle" + turnDegrees + ":" + ((180 / Math.PI) * (destination.getHeading()) + ":"
							+ ((180 / Math.PI) * currentPosition.getHeading())));

		}

	}

	private void exec_SETPOSE_ALGO() {
		monitor.writeControlComment("setPose");
		if (mutexSetPose == 0) {
			destination.setLocation(1.8f, 0.6f);
			destination.setHeading((float) (navigation.getPose().getHeading() + (Math.PI) / 2D));

			mutexSetPose = 1;
		}

		if (desiredHeading == false) {
			turnToLocation();
		}

		if (desiredHeading == true && finalLocation == false) {
			if (mutexGetHeading == 0) {
				phi = Math.toDegrees((currentPosition.getHeading()));
				mutexGetHeading = 1;
			}
			float distance = currentPosition.distanceTo(destination.getLocation());
			if (Math.abs(distance) > 0.1) {
				float x = currentPosition.getX();
				float y = currentPosition.getY();
				double calcPos = ((y - yStart) * Math.cos(phi)) + ((x - xStart) * Math.sin(phi));

				double kd = calcPos - calculatedPositionBefore;
				double w = 80 * calcPos + 5 * kd;
				calculatedPositionBefore = calcPos;
				xStart = x;
				yStart = y;
				monitor.writeControlComment("CooTrans" + " : " + distance + ":" + w + ": " + calcPos + ":" + kd);
				monitor.writeControlComment("pose" + phi + ":" + currentPosition.getX() + ":" + currentPosition.getY());
				// driveNEW(0.1, w);
				drive(9, -w);
			} else {
				stop();
				mutexTurn = 0;
				finalLocation = true;

			}

		} /*
			 * else { if (desiredHeading == true && finalLocation == true &&
			 * finalDestination == false) { turnDefinedAngle(); } else {stop();}
			 * 
			 * }
			 */
		if ((desiredHeading == true) && (finalLocation == true) && (finalDestination == false)) {
			turnDefinedAngle();
		}

	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		if (parking == false) {
			ParkBackwords();
		}
		if (parking == true) {

			ParkOut();
		}
	}

	private void exec_INACTIVE() {

		this.stop();
	}

	private float calculatePoint(float x, float y) {
		// need to know exact function
		return 0;

	}

	private float calculatePath(float x, float y) {
		// need to know 1st derivation of the function above
		return 0;
	}

	/**
	 * DRIVING along black line Minimalbeispiel Linienverfolgung fuer gegebene
	 * Werte 0,1,2 white = 0, black = 2, grey = 1
	 */

	private void exec_LINECTRL_ALGO() {
		setPose(navigation.getPose());

		leftMotor.forward();
		rightMotor.forward();

		// monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		// monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
		differenceErrorL = perception.getLeftRough() - perception.getRightRough();
		integralLine = integralLine + differenceErrorL;
		pdline = differenceErrorL - lastErrorL;
		lastErrorL = differenceErrorL;
		if (Math.abs(differenceErrorL) == 0) {
			integralLine = 0;
		}
		if (Math.abs(differenceErrorL) > 10) {
			if (differenceErrorL > 10) {
				// drive(0, -30);
				leftMotor.setPower((int) (40));
				rightMotor.setPower(-1);
				differenceErrorL = perception.getLeftRough() - perception.getRightRough();

			}
			if (differenceErrorL < -10) {
				leftMotor.setPower(-1);
				rightMotor.setPower((int) (40));
				differenceErrorL = perception.getLeftRough() - perception.getRightRough();

			}
		} else {
			ksline = (int) ((Kpline * differenceErrorL) + (Kdline * pdline) + KiLine * integralLine);

			int start;
			if (offset > 0) {
				start = offset - ((perception.getLSlwhiteValue() + perception.getLSlblackValue()) / 2
						- (perception.getRightRough() + perception.getLeftRough()) / 2);
				rightMotor.setPower(start - ksline);// - speed);
				leftMotor.setPower(start + ksline);

			} else {
				start = offset + ((perception.getLSlwhiteValue() + perception.getLSlblackValue()) / 2
						- (perception.getRightRough() + perception.getLeftRough()) / 2);
				rightMotor.setPower(start + (ksline / 2));// - speed);
				leftMotor.setPower(start - (ksline / 2));

			}

			monitor.writeControlComment("PIDL1" + ksline + "start" + start);

		}

	}

	private void minischub() {
		rightMotor.setPower(18); // Um ein abstoppen auf der Linie zu
		// verhindern wird mit minimalem speed
		// weitergefahren,danach sofort gebraked
		leftMotor.setPower(18);

		// brake();
	}

	private void stop() {
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

	private void resetAdd() {
		addL = 0;
		addR = 0;
		radsummeL = 0;
	}

	/**
	 * function to handle all avoidings/changes of driving direction if winkel
	 * == 0, the car will drive Straight on with a given velocity if velocity ==
	 * 0 the car will turn on his place while it is interrupted from another
	 * function if winkel > 0 left turn if winkel < 0 right turn if winkel &&
	 * velocity != 0 the car will rotate in a calculated radius
	 */

	public void drive(double v, double w) {
		AngleDifferenceMeasurement admL = encoderLeft.getEncoderMeasurement();
		AngleDifferenceMeasurement admR = encoderRight.getEncoderMeasurement();
	
		double kd = 0.5;// 0.2;// 2;
		double errorL = 0;
		double errorR = 0;

		// ###### 1. Schritt Kinematikberechnung ##### ///
		if (w != 0) {
			double angular_speed = ((4 / 15D) * w) / 0.1;
			w = angular_speed;
		}
		w = Math.toRadians(w);

		double velocityL = v - (10 / 2.8D) * w;
		double velocityR = v + (10 / 2.8D) * w;

		// ####### 2. Schritt Powerwerte berechnen #####//
		monitor.writeControlComment("Speed" + velocityL + velocityR);
		if (velocityL > 0) {
			powerL = ((2.125 * (velocityL)) + 9.6D);
		} else {
			powerL = ((1.4 * (velocityL)) - 11.94D);
		}
		if (velocityR > 0) {
			powerR = ((2.125 * (velocityR)) + 9.6);
		} else {
			powerR = ((1.4 * (velocityR)) - 11.94D);

		}
		double timeL = admL.getDeltaT();
		double timeR = admL.getDeltaT();
		double lsum = admL.getAngleSum();
		double rsum = admR.getAngleSum();
		radsummeL = radsummeL + lsum;
		// ###### 3. Schritt PID Regelung dazuaddieren ####//
		if (timeL != 0 && timeR != 0) {
			timeL = timeL / 1000;
			timeR = timeR / 1000;
			tempSpeedLeft = ((lsum / 360D) * 17.59D) / timeL;
			// 2*Math.PI*2.8D;//admL.getDeltaT()*5;
			tempSpeedRight = ((rsum / 360D) * 17.59D) / timeR;

			errorL = tempSpeedLeft - (velocityL);
			errorR = tempSpeedRight - (velocityR);
			if (Math.abs(errorL) < 0.1) {
				integralL = 0;
			} else {
				integralL = integralL + errorL;
			}
			if (Math.abs(errorR) < 0.1) {
				integralR = 0;
			} else {
				integralR = integralR + errorR;
			}
			differenceL = errorL - lastErrorLinks;
			differenceR = errorR - lastErrorR;
			if (errorL > 0) {
				double kp_l = 1.8;// 0.75;// 5;
				double kp_r = 1.92;// 0.75;// 5;
				double ki_r = 0.12;// 0.01;// 0.2;
				double ki_l = 0.1;
				pidL = kp_l * errorL + ki_l * integralL + kd * differenceL;
				pidR = kp_r * errorR + ki_r * integralR + kd * differenceL;

			} else {
				double ki_r = 0.1;// 0.01;// 0.2;
				double ki_l = 0.12;
				double kp_r = 1.8;// 0.75;// 5;
				double kp_l = 1.92;// 0.75;// 5;
				pidL = kp_l * errorL + ki_l * integralL + kd * differenceL;
				pidR = kp_r * errorR + ki_r * integralR + kd * differenceL;

			}

			lastErrorLinks = errorL;
			lastErrorR = errorR;
		}
		// ##### fahren ####///
		leftMotor.forward();
		rightMotor.forward();

		leftMotor.setPower((int) (powerL) - (int) (pidL));
		rightMotor.setPower((int) powerR - (int) pidR);

		monitor.writeControlComment("drive" + ": " + (int) powerL + ":" + (int) powerR + ":" + (int) pidL + ":"
				+ (int) pidR + ":" + velocityL + ":" + velocityR + ":" + tempSpeedLeft + ":" + tempSpeedRight + ":"
				+ errorL + ":" + lsum + ":" + rsum + ":" + timeL);
	}

	public void ParkForward() {
		AngleDifferenceMeasurement admL = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement admR = perception.getControlRightEncoder().getEncoderMeasurement();
		admL.setDeltaT(100);
		admR.setDeltaT(100);
		if (mutex_out == 0) {
			time = System.currentTimeMillis();
			mutex_out = 1;
		}
		long diff = System.currentTimeMillis() - time;
		/*
		 * if (x > 0.5) { stop(); return; }
		 */
		if ((x < 6)) {
			double a = 0.004;
			double b = -0.03;
			double v = 1;

			if ((x > 4) && currentPosition.getHeading() >= destination.getHeading()) {
				stop();
			} else if (diff % 1000 < 109 && (parking == false)) {
				x = x + 1;
			}
			velo = Math.sqrt(v * v + Math.pow((3 * a * v * v * v * x * x + 2 * b * v * v * x), 2));
			w = 2 * (3 * a * x + b) / (Math.pow((3 * a * x * x + 2 * b * x), 2) + 1);
			// if (currentPosition.getHeading()!=destination.getHeading()){
			drive((velo * 5), (w * 600));

		} else {
			stop();
			parking = true;
		}
		// }

		// if(currentPosition.getHeading())
		monitor.writeControlComment("Time " + diff + ":" + System.currentTimeMillis() + ":" + time);
		monitor.writeControlComment("ParkIN" + ":" + x + ":" + w + ":" + velo);
		/// Timer für alle 0.1 s

	}

	public void ParkOut() {
		AngleDifferenceMeasurement admL = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement admR = perception.getControlRightEncoder().getEncoderMeasurement();
		succesfullyParkedOut = false;
		admL.setDeltaT(100);
		admR.setDeltaT(100);
		if (mutex == 0) {
			time = System.currentTimeMillis();
			destination.setHeading(currentPosition.getHeading());
			mutex = 1;
		}
		long diff = System.currentTimeMillis() - time;
		if (x > 5) {
			stop();
			if (!finalDestination) {
				turnDefinedAngle();
			} else {
				stop();
				
				succesfullyParkedOut = true;
				parking = false;
				return;
			
			} 
			}
		else
		if (diff % 1000 < 109) {
			x = x + 1;

			double a = -0.004;
			double b = 0.03;
			double v = 1;
			velo = Math.sqrt(v * v + Math.pow((3 * a * v * v * v * x * x + 2 * b * v * v * x), 2));
			w = 2 * (3 * a * x + b) / (Math.pow((3 * a * x * x + 2 * b * x), 2) + 1);
			drive((velo * 7), (w * 700));

		}
		// if(currentPosition.getHeading())
		monitor.writeControlComment("Time " + diff + ":" + System.currentTimeMillis() + ":" + time);
		monitor.writeControlComment("ParkIN" + ":" + x + ":" + w + ":" + velo);
		/// Timer für alle 0.1 s
	}

	public void ParkBackwords() {
		AngleDifferenceMeasurement admL = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement admR = perception.getControlRightEncoder().getEncoderMeasurement();
		admL.setDeltaT(100);
		admR.setDeltaT(100);

		succesfullyParkedIn = false;
		if (mutex == 0) {
			time = System.currentTimeMillis();
			destination.setHeading( currentPosition.getHeading());
			
			mutex = 1;
		}
		long diff = System.currentTimeMillis() - time;
		if (x_back < -5) {
			succesfullyParkedIn = true;
			parking = true;
			if (!finalDestination) {
				turnDefinedAngle();
			} else {
				setCtrlMode(IControl.ControlMode.INACTIVE); 
				stop();
				stop();
			}
		
			try {
				Thread.sleep(2000);
			} catch (Exception e) {
			} 
			
			return;
		}

		else
		if ( System.currentTimeMillis() - time>1000) {
			x_back = x_back - 1;
			mutex = 0; }
			double a = -0.004;
			double b = 0.03;
			double v = 1;
			velo = Math.sqrt(v * v + Math.pow((3 * a * v * v * v * x_back * x_back + 2 * b * v * v * (-x_back)), 2));
			w = 2 * (3 * a * (-x_back) + b) / (Math.pow((3 * a * x_back * x_back + 2 * b * (-x_back)), 2) + 1);
			drive(-(velo * 8), (w * 800));

		
		// if(currentPosition.getHeading())
		monitor.writeControlComment("Time " + diff + ":" + System.currentTimeMillis() + ":" + time);
		monitor.writeControlComment("ParkNOW" + ":" + x_back + ":" + w + ":" + -3 * velo);
		/// Timer für alle 0.1 s
	}

	private void turn90(float heading, double w) {
		if (currentPosition.getHeading() < (heading + Math.PI / 2)) {
			drive(0, w);
			monitor.writeControlComment("turn90" + heading + currentPosition.getHeading());
		} else {
			turn90 = true;
			stop();
		}
		return;

	}

	private void testSequenz1() {
		Point start = new Point(0.5f,0.0f);
		
		if (!driven10) {
			Double way = (radsummeL / 360D) * 17.59D;
			if ((way) > 145) {
				driven10 = true;
				stop();
			} else {
				drive(10, 0);
			}
			monitor.writeControlComment("Distance" + ":" + way);
		}
		if (driven10 && !turn15) {
			if (turn90_mutex == 0) {
				call = currentPosition.getHeading();
				turn90_mutex = 1;
			}
			if (!turn90) {
				turn90(call, 15);
			} else {
				resetAdd();
				turn15 = true;
				stop();
			}
		} 
		if (driven10 && turn15 && !driven5) {
			Double way = (radsummeL / 360D) * 17.59D;
			if ((way) > 28) {
				turn90_mutex = 0;
				turn90 =false;
				driven5 = true;
				stop();
			} else {
				drive(5, 0);
			}
			monitor.writeControlComment("Distance" + ":" + way);
		}
		if (driven5 && !turn30) {
			if (turn90_mutex == 0) {
				call = currentPosition.getHeading();
				turn90_mutex = 1;
			}
			if (!turn90) {
				turn90(call, 30);
			} else {
				resetAdd();
				turn30 = true;
				stop();
			}

		}
		if(driven5 &&turn30 ){
			setBackwords(false);
			exec_LINECTRL_ALGO();
		if(currentPosition.distanceTo(start)<0.04){
			stop();
		}
		}
		/*
		Button.ENTER.waitForPressAndRelease();
		LCD.clear();
		LCD.drawString("Linefollower", 0, 0); */

	}
}
