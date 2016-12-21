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
	float Kpline = 1.1f;
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
	float phi = currentPositionDegrees.getHeading();
	double calculatedPositionBefore = 0;
	boolean finalDestination = false;
	boolean finalLocation = false;
	int mutexSetPose = 0;

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
	 * Function to do Linefollowing backwords to avoid collisions
	 * must be called from Guidance !!!!!  */ 
	public void setBackwords(boolean back) {
		if (back == true) {
			this.offset = -30;
		}
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
		//	setBackwords(true);
			update_LINECTRL_Parameter();
			exec_LINECTRL_ALGO();
			// testdrive();
			// update_SETPOSE_Parameter();
			// exec_SETPOSE_ALGO();
			//driveNEW(0.1, -35);
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
				drive(0, -30);

			} else {
				if (degree > 5) {
					drive(0, 30);
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

			double turnDegrees;
			turnDegrees = ((180 / Math.PI) * (destination.getHeading()) -((180 / Math.PI) * currentPosition.getHeading()));
			// ######## to avoid an overflow ########
			if (turnDegrees < -180) {
				turnDegrees = turnDegrees + 360;
			} else if (turnDegrees > 180) {
				turnDegrees = turnDegrees - 360;
			}
			// ######## turn until MAth.abs(angle) =1 #####
			if (turnDegrees > 5) {
				drive(0, 20);
			} else if (turnDegrees < -5) {
				drive(0, -20);
			} else {
				// ### break the while because we have reached the desired
				// Heading
				stop();
				finalDestination = true;
				return;
			}
			monitor.writeControlComment("TurnDefinedAngle" + 
			turnDegrees
			+":"+((180 / Math.PI) * (destination.getHeading())
			+":"+((180 / Math.PI) * currentPosition.getHeading())));
		}

	}

	private void exec_SETPOSE_ALGO() {
		monitor.writeControlComment("setPose");
		if (mutexSetPose == 0) {
			destination.setLocation(0.4557506f, -0.212861f);
			destination.setHeading((float) (navigation.getPose().getHeading()));
			mutexSetPose = 1;
		}

		if (desiredHeading == false) {
			turnToLocation();
		}

		if (desiredHeading == true && finalLocation == false) {
			float distance = currentPosition.distanceTo(destination.getLocation());
			if (Math.abs(distance) > 0.05) {

				double calcPos = ((currentPosition.getY() - yStart) * Math.sin(phi))
						+ ((currentPosition.getX() - xStart) * Math.cos(phi));

				double kd = calcPos - calculatedPositionBefore;
				double w = 100 * calcPos  + 5 * kd;
				calculatedPositionBefore = calcPos;
				monitor.writeControlComment("CooTrans" + " : " + distance + ":" + w + ": " + calcPos + ":" + kd);
				monitor.writeControlComment("pose" + currentPosition.getX() + ":" + currentPosition.getY());
				driveNEW(0.1, w);
				

			} else {
				stop();
				mutexTurn=0;
				finalLocation = true;
				
			} /*
				 * selse { if (distance <= 0.1) { finalLocation = true;
				 * turnDefinedAngle(); } else { stop(); } // need to turn to the
				 * right heading }
				 */
		}

		if (desiredHeading == true && finalLocation == true) {
			turnDefinedAngle();
		}

	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		Pose neu = new Pose();
		neu.setLocation(currentPositionDegrees.getX() + 2, 5);
		if (currentPositionDegrees.relativeBearing(neu.getLocation()) < -1) {
			drive(0, 30);
		}
		if (currentPositionDegrees.relativeBearing(neu.getLocation()) > 1) {
			drive(0, -30);
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
		if (Math.abs(differenceErrorL) > 15) {
			if (differenceErrorL > 15) {
				// drive(0, -30);
				leftMotor.setPower((int) (37));
				rightMotor.setPower(0);
				differenceErrorL = perception.getLeftRough() - perception.getRightRough();

			}
			if (differenceErrorL < -15) {
				leftMotor.setPower(0);
				rightMotor.setPower((int) (37));
				differenceErrorL = perception.getLeftRough() - perception.getRightRough();

			}
		} else {
			ksline = (int) ((Kpline * differenceErrorL) + (Kdline * pdline) + KiLine * integralLine);
			
			int start;
			if (offset >0 ) {
			start = offset - ((perception.getLSlwhiteValue() + perception.getLSlblackValue()) / 2
					- (perception.getRightRough() + perception.getLeftRough()) / 2);
			rightMotor.setPower(start - ksline);// - speed);
			leftMotor.setPower(start + ksline);
			
			} else {
			start = offset + ((perception.getLSlwhiteValue() + perception.getLSlblackValue()) / 2
					- (perception.getRightRough() + perception.getLeftRough()) / 2);
			rightMotor.setPower(start + (ksline/2));// - speed);
			leftMotor.setPower(start - (ksline/2));
			 
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

	// Aufgabe 3.2
	private void testdrive() {
		if (mutex == 0) { // count ist global auf 0 initialisiert
			time = System.currentTimeMillis();
			time = time + 30001;
			mutex++;
		}

		if (time != 0) {
			// ######### drive 10cm/s for 15s ###################
			if (System.currentTimeMillis() < (time - 28000)) {
				drive(30, 0);

			} else {
				if (System.currentTimeMillis() < (time - 19000)) {
					drive(0, -30);

				} else {
					mutexTurn = 0;
					leftMotor.stop();
					rightMotor.stop();
				}
			}
		}

	}

	/**
	 * function to handle all avoidings/changes of driving direction if winkel
	 * == 0, the car will drive Straight on with a given velocity if velocity ==
	 * 0 the car will turn on his place while it is interrupted from another
	 * function if winkel > 0 left turn if winkel < 0 right turn if winkel &&
	 * velocity != 0 the car will rotate in a calculated radius
	 */
	private void drive(double velocity, double winkel) {
		AngleDifferenceMeasurement admL = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement admR = perception.getControlRightEncoder().getEncoderMeasurement();
		addL = addL + admL.getAngleSum();
		addR = addR + admR.getAngleSum();
		leftMotor.forward();
		rightMotor.forward();
		admL.setDeltaT(50);
		admR.setDeltaT(50);
		speedL = (((1 / 3D) * Math.abs(winkel)) + 20D);// 30;// (((1 / 5D) *
														// winkel) +
		// 13D);
		double Kp = 0.2;
		double Ki_line = 0.1;
		double Kd_line = 0;
		// addL < 150 dreht 90 ° -> 600 dreht 360
		int wheelDistance = 0; // nochmal unbedingt nachmessen
		double radiusM;
		double velTurn = (((7 / 5D) * (velocity * 100D)) + 20D);

		if (winkel == 0) { // do something to drive straight forward
			radiusM = velocity;// velTurn;
			wheelDistance = 0;
		} else {
			radiusM = velocity / winkel;
			if (radiusM == 0) {

				if (mutexTurn == 0) {
					resetAdd();
					mutexTurn = 1;
				}
				if (winkel > 0) { // Turn lEft
					tempErrorVWLeft = addL + addR; // one positiv and one//
													// negativ
					if (tempErrorVWLeft == 0) {
						integral = 0;
					} // Do an antiwindupcontrol
					integral = integral + tempErrorVWLeft;
					difference = tempErrorVWLeft - lastErrorVWLeft;
					double ks = (Kp * tempErrorVWLeft) + (Ki_line * integral) + (Kd_line * difference);
					leftMotor.setPower((int) (-speedL - ks));
					rightMotor.setPower((int) (speedL - ks));

					monitor.writeControlComment("TurnL" + speedL + ":" + ks);
				}

				if (winkel < 0) { // Turn Right
					tempErrorVWLeft = addL + addR; // one positiv and one //
													// negativ

					if (tempErrorVWLeft == 0) {
						integral = 0;
					} // Do an antiwindupcontrol
					integral = integral + tempErrorVWLeft;
					difference = tempErrorVWLeft - lastErrorVWLeft;
					double ks = (Kp * tempErrorVWLeft) + (Ki_line * integral) + (Kd_line * difference);
					leftMotor.setPower((int) (speedL - ks));
					rightMotor.setPower((int) (-speedL - ks));

					monitor.writeControlComment("TurnR" + speedL + ":" + ks);
				}

				// ": " + tempErrorVWLeft);
				// monitor.writeControlComment("TurnL" + addL + ":" + fixAddL +
				// ": " + tempErrorVWLeft);

			} else {
				wheelDistance = 10;
				double velocityL;
				double velocityR;
				if (radiusM > 0) {
					velocityL = ((radiusM - (wheelDistance / 2D)) * (velocity / radiusM));
					velocityR = (radiusM + (wheelDistance / 2D)) * (velocity / radiusM);
				} else {
					radiusM = Math.abs(radiusM);
					velocityL = ((radiusM + (wheelDistance / 2D)) * (velocity / radiusM));
					velocityR = (radiusM - (wheelDistance / 2D)) * (velocity / radiusM);
				}

				leftMotor.setPower((int) velocityL);
				rightMotor.setPower((int) velocityR);
				monitor.writeControlComment("radius" + velocityL + ":" + velocityR + ":" + radiusM);
			}
		}

		if (velocity != 0 && winkel == 0) {

			if (mutexStraight == 0) {
				resetAdd();
				mutexStraight = 1;
				mutexTurn = 0;
			}

			double KpStraight = 0.9;
			double KiStraight = 0.17;
			double KdStraight = 0;
			tempErrorVWLeft = addL - addR;
			if (tempErrorVWLeft == 0) {
				integral = 0;
			} // Do an antiwindupcontrol
			if (tempErrorVWLeft > 8) {
				tempErrorVWLeft = 8;
			} // Fehlerbegrenzung für Peak
			integral = integral + tempErrorVWLeft;
			difference = tempErrorVWLeft - lastErrorVWLeft;
			double ks = (KpStraight * tempErrorVWLeft) + (KiStraight * integral) + (KdStraight * difference);
			int left = (int) (35 - ks);
			int right = (int) (35 + ks);
			leftMotor.setPower(left);
			rightMotor.setPower(right);
			monitor.writeControlComment("links" + ks + ":" + left + ":" + right + "temError" + tempErrorVWLeft);

			lastErrorVWLeft = tempErrorVWLeft;
		}

	}

	private void driveNEW(double v, double w) {
		double wl;
		double wr;
		leftMotor.forward();
		rightMotor.forward();

		if (w > 0) { //links tunr
			w = Math.toRadians(Math.abs(w));
			wl = (900 * (0.3875) * v) - (80 * 0.129D * w);
			wr = (900 * (0.3875) * v) + (80 * 0.129D * w);
		} else { // rechts turn
			w = Math.toRadians(Math.abs(w));
			wl = (900 * (0.3875) * v) + (80 * 0.129D * w);
			wr = (900 * (0.3875) * v) - (80 * 0.129D * w);
		}
		monitor.writeControlComment("driveNew" + w + ":" + wl + ":" + wr);
		leftMotor.setPower((int) wl);
		rightMotor.setPower((int) wr);

		// PID Regler for every wheel !!!!!!
	}

	private void resetAdd() {
		addL = 0;
		addR = 0;
	}

}
