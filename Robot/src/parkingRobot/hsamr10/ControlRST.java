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

	int lastTime = 0;

	double currentDistance = 0.0;
	double Distance = 0.0;

	// Linefollower
	int lastErrorL = 0;
	int differenceErrorL = 1;
	int integralLine = 0;
	float Kpline = 1.1f;
	float KiLine = 0.1f;
	float Kdline = 0.9f;
	int pdline =0;
	int ksline =0;
	
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
	double integral = 0;
	double difference = 0;
	double fixAddL = 0;
	double fixAddR = 0;
	int mutexTurn = 0;
	int mutexStraight = 0;

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
			update_LINECTRL_Parameter();
			exec_LINECTRL_ALGO();
			// testdrive();
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
	}

	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter() {
		// Aufgabe 3.4
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

	private void exec_SETPOSE_ALGO() {
		// Aufgabe 3.3
	}

	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO() {
		// Aufgabe 3.4
	}

	private void exec_INACTIVE() {
		this.stop();
	}

	/**
	 * DRIVING along black line Minimalbeispiel Linienverfolgung fuer gegebene
	 * Werte 0,1,2 white = 0, black = 2, grey = 1
	 */

	private void exec_LINECTRL_ALGO() {
		leftMotor.forward();
		rightMotor.forward();
		
	//	monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
	//	monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);
		differenceErrorL = perception.getLeftRough() - perception.getRightRough();
		integralLine = integralLine + differenceErrorL;
		pdline = differenceErrorL - lastErrorL;
		lastErrorL = differenceErrorL;
		if(Math.abs(differenceErrorL) < 4  ){integralLine = 0;}
		if (differenceErrorL > 15) {
				//drive(0, -30);
				leftMotor.setPower(40);
				rightMotor.setPower(0);
		} else {
			if (differenceErrorL < -15) {
				leftMotor.setPower(0);
				rightMotor.setPower(40);
			} else {
				ksline = (int) ((Kpline * differenceErrorL) + (Kdline * pdline)+ KiLine*integralLine);
				rightMotor.setPower(30 - ksline);// - speed);
				leftMotor.setPower(30 + ksline);
		//		monitor.writeControlComment("PIDL1" + ksline + "differenceErrorL" + differenceErrorL);
			}
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

	private void drive(double velocity, double winkel) {
		AngleDifferenceMeasurement admL = perception.getControlLeftEncoder().getEncoderMeasurement();
		AngleDifferenceMeasurement admR = perception.getControlRightEncoder().getEncoderMeasurement();
		addL = addL + admL.getAngleSum();
		addR = addR + admR.getAngleSum();
		leftMotor.forward();
		rightMotor.forward();
		admL.setDeltaT(50);
		admR.setDeltaT(50);
		speedL = 30;// (((1 / 5D) * winkel) + 13D);
		double Kp = 0.2;
		double Ki_line = 0.1;
		double Kd_line = 0;

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
					mutexStraight = 0;
					mutexTurn = 1;
				}
				if (winkel > 0) { // Turn lEft
					if (addR < 150) {
						tempErrorVWLeft = addL + addR; // one positiv and one
														// negativ
						if (tempErrorVWLeft == 0) {
							integral = 0;
						} // Do an antiwindupcontrol
						integral = integral + tempErrorVWLeft;
						difference = tempErrorVWLeft - lastErrorVWLeft;
						double ks = (Kp * tempErrorVWLeft) + (Ki_line * integral) + (Kd_line * difference);

						leftMotor.setPower((int) (-speedL - ks));
						rightMotor.setPower((int) (speedL - ks));
					} else {
						leftMotor.setPower(0);
						rightMotor.setPower(0);

					}

				}
				if (winkel < 0) { // Turn lEft
					if (addL < 150) {
						tempErrorVWLeft = addL + addR; // one positiv and one
														// negativ
						if (tempErrorVWLeft == 0) {
							integral = 0;
						} // Do an antiwindupcontrol
						integral = integral + tempErrorVWLeft;
						difference = tempErrorVWLeft - lastErrorVWLeft;
						double ks = (Kp * tempErrorVWLeft) + (Ki_line * integral) + (Kd_line * difference);

						leftMotor.setPower((int) (speedL - ks));
						rightMotor.setPower((int) (-speedL - ks));
					} else {
						leftMotor.setPower(0);
						rightMotor.setPower(0);

					}

				}
				monitor.writeControlComment("TurnR" + addR + ":" + fixAddR + ": " + tempErrorVWLeft);
				monitor.writeControlComment("TurnL" + addL + ":" + fixAddL + ": " + tempErrorVWLeft);

			} else {
				wheelDistance = 10;
			}
		}
		double velocityL = ((radiusM - (wheelDistance / 2)) * (velocity / radiusM)) - 1;
		double velocityR = (radiusM + (wheelDistance / 2)) * (velocity / radiusM);
		if (velocity != 0) {
			if (mutexStraight == 0) {
				resetAdd();
				mutexTurn = 0;
				mutexStraight++;
			}

			double KpStraight = 0.9;
			double KiStraight = 0.17;
			double KdStraight = 0;
			tempErrorVWLeft = addL - addR;
			if (tempErrorVWLeft == 0) {
				integral = 0;
			} // Do an antiwindupcontrol
			integral = integral + tempErrorVWLeft;
			difference = tempErrorVWLeft - lastErrorVWLeft;
			double ks = (KpStraight * tempErrorVWLeft) + (KiStraight * integral) + (KdStraight * difference);
			int left = (int) (35 - ks);
			int right = (int) (35 + ks);
			leftMotor.setPower(left);
			rightMotor.setPower(right);
			monitor.writeControlComment("links" + ks + ":" + left + ":" + right);

			lastErrorVWLeft = tempErrorVWLeft;
		}
	}

	private void resetAdd() {
		addL = 0;
		addR = 0;
	}
}
