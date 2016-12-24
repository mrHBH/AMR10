package parkingRobot.hsamr10;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.hsamr10.ControlRST;
import parkingRobot.hsamr10.HmiPLT;
import parkingRobot.hsamr10.NavigationAT;
import parkingRobot.hsamr10.PerceptionPMP;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;


/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
 * with specialization 'automation, measurement and control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego NXT system witch can perform
 * parking maneuvers on an predefined course. To fulfill the interdisciplinary aspect of this project the software
 * structure is divided in 5 parts: human machine interface, guidance, control, perception and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be controlled by one or more finite
 * state machines (FSM). It may be advantageous to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be realized in one main module class.
 * Every class (except guidance) has additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module class thread is only handled in a
 * synchronized context to avoid inconsistent or corrupt data!
 */
public class GuidanceAT {
	
	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		SCOUT,
		/**
		 * indicates that robot is idling
		 */
		INACTIVE,
		/**
		 * indicates that robot is searching for new parking places to park as soon as possible
		 */
		PARKNOW,
		/**
		 * indicates that robot is following the line to reach the selected slot and will initiate the parking operation there.
		 */
		PARKTHIS,
		/**
		 * indicates the Robot is performing the leaving maneuver and  will go to scout mode upon termination
		 */
		AUSPARKEN,
		/**
		 * indicates that the program is terminating the Bluetooth connection and shutting down the robot. 
		 */
		EXIT
		
	}
	public enum CurrentSubState {
		/**
		 * The Robot has detected a collision and is trying to avoid it 
		 */
		AVOIDINGCOLLISION,
		/**
		 * The Robot has lost the line and is performing a special maneuver to find it again.
         */
		FINDINGTRACK,
		/**
		 * The Robot is performing the measurement maneuver.
		 */
		MEASURING,
		/**
		 * The robot can perform the required task without needing to go to a Substate.
		 */
		NOSUBSTATE
	}
	
	
	
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.INACTIVE;
	protected static CurrentSubState currentSubState = CurrentSubState.NOSUBSTATE;
	protected static CurrentSubState lastSubState = CurrentSubState.NOSUBSTATE;

	
	/**
	 * one line of the map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * This documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(  0,  0, 180,  0);
	static Line line1 = new Line(180,  0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30,  30, 30);
	static Line line5 = new Line( 30, 30,  30, 60);
	static Line line6 = new Line( 30, 60,   0, 60);
	static Line line7 = new Line(  0, 60,   0,  0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * All above defined lines are bundled in this array and to form the course map.
	 */
	static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};
	
	
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	
	public static void main(String[] args) throws Exception {		
        currentStatus = CurrentStatus.INACTIVE;
        lastStatus    = CurrentStatus.EXIT;
        lastSubState =CurrentSubState.NOSUBSTATE;		 	
        currentSubState=CurrentSubState.NOSUBSTATE;
		/** Output Channel A is broken, so we have to use B and C instead
		 * For better results we have to cross the outputs**/
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.C);
		
		IMonitor monitor = new Monitor();
		
		IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
		perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT(perception, monitor);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control, monitor);
		
		navigation.setMap(map);
		monitor.startLogging();
		/**Collision threshold is a variable that sets a threshold distance. If the front IR sensors detects a value lower than the 
		 * threshold , a collision is then detected.
		 * 
		 */
		double CollisionThreshod=0;
		/**Detecting is boolean , that when true it activates the detection of potential parking slots .
		 * 
		 */
		double currentDistance=0;
		boolean CollisionDetected=false;
		boolean LostTrack=false;
		boolean FoundPotentialSlot=false;
		
				
		while(true) {
			showData(navigation, perception);
			//The RObot can only go to one of the main states if it is not currently inside any substate
			if ( ( currentSubState==CurrentSubState.NOSUBSTATE) || (currentStatus!=CurrentStatus.EXIT) /*|| (currentStatus!=CurrentStatus.INACTIVE)*/)
			
			{
				
        	switch ( currentStatus )
        	{
				case SCOUT:
					// MONITOR (example)
				//	monitor.writeGuidanceComment(Double.toString(perception.getFrontSensorDistance()));
					
					//Into action
					if ( lastStatus != CurrentStatus.SCOUT || lastSubState !=CurrentSubState.NOSUBSTATE ){
						control.setCtrlMode(ControlMode.LINE_CTRL);
                        CollisionThreshod=7;
						navigation.setDetectionState(true);
					}
					
					//While action				
					{
						//nothing to do here
					}					
					
					//State transition check
					lastStatus = currentStatus;
					
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
						currentStatus = CurrentStatus.INACTIVE;	
					}else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
						currentStatus = CurrentStatus.EXIT;}
					
					//if collisiondetected is set to true in the main loop , the robot will go to avoiding collision Substate 
				    if (CollisionDetected){
						
				
						currentSubState=CurrentSubState.AVOIDINGCOLLISION; 
					}
				    
				    // Collision detected  and going to the avoiding collision substate
					if (LostTrack) { 
						currentSubState=CurrentSubState.FINDINGTRACK; } // Lost The line and the robot is trying to find it 
					if ( FoundPotentialSlot ){
						currentSubState=CurrentSubState.MEASURING;
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.SCOUT ){
						//nothing to do here
					}
					break;				
				case INACTIVE:
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
					}
					
					//While action
					{
						//nothing to do here
					}
					
					//State transition check
					lastStatus = currentStatus;
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
						currentStatus = CurrentStatus.SCOUT;						
					}else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.SCOUT;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
						currentStatus = CurrentStatus.EXIT;
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.INACTIVE ){
						//nothing to do here
					}					
					break;
				case EXIT:
					hmi.disconnect();
					/** NOTE: RESERVED FOR FUTURE DEVELOPMENT (PLEASE DO NOT CHANGE)
					// monitor.sendOfflineLog();
					*/
					monitor.stopLogging();
					System.exit(0);
					break;
				case PARKTHIS:
					//CHECK CURRENT POSITION ,LINE and POSE 
					//IF not desired then GO to scout MODE using the 'executing' BOOLEAN until the required POSITON IS REACHED
					//IF THE required position is reached then generate the path and send it to control 
					//set threshold to 5 
					//if parking is successful go to INACTIVE state
					
			default:
				break;
        	
			}
        	
        }
			  switch ( currentSubState )
			  
        	{ case AVOIDINGCOLLISION:
        		
        		/** Avoiding Collision 
        		 * the Robot stops when an obstacle within "CollisionThreshold" is detected . 
        		 * if the obstacle approaches beyond the distance 2 from the front , The robot will step backwards
        		 * if the obstacle is static and still nearer than the  "collision threshold" the robots stops and waits 
        		 * if the collision is cleared , the robot leaves "avoiding Collision" to "NOSUBSTATE" 
          		 */
        		

				// Into action
        		
        		if (lastSubState!=currentSubState){
        			
    				control.setCtrlMode(ControlMode.INACTIVE);
    				lastSubState=currentSubState;
        			}
        		
        		//Transition Check 
        		
        			
        		
				currentDistance=perception.getFrontSensorDistance();      // Updates the front sensor value ;
            	if (currentDistance>=2 && currentDistance<CollisionThreshod){
             		control.setCtrlMode(ControlMode.INACTIVE);
             		
             	}
            	
            	
             	else if (currentDistance<2) {  // The Collision is approaching : step backwards
             		
          		 control.setBackwords(true); // this enables the backward line follower
			     control.setCtrlMode(ControlMode.LINE_CTRL);
             	
             		
             	}
            	 
             	else {            	// The Collision is cleared : exit the substate 

             		CollisionDetected=false;
           		    control.setBackwords(false); // this disables the backward line follower 
				    control.setCtrlMode(ControlMode.LINE_CTRL);
                    currentSubState=CurrentSubState.NOSUBSTATE;}
             
				break;
				
				case NOSUBSTATE : 
					lastSubState=currentSubState;
					break;
					
				default:
	        		break;
        		}
			
		    if (perception.getFrontSensorDistance()<CollisionThreshod){CollisionDetected=true;}
			     	Thread.sleep(10);        	
		}
	}
	
	
	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return GuidanceAT.currentStatus;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception){
		LCD.clear();	
		//perception.showSensorData();
		LCD.drawString("Front: "+perception.getFrontSensorDistance(), 0, 4);	
		LCD.drawString("STATE: "+currentStatus, 0, 0);
		LCD.drawString("SUBSTATE: "+currentSubState, 0, 2);
		

//		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);

//		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
//		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);
//		
//		perception.showSensorData();
		
//    	if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
//			LCD.drawString("HMI Mode SCOUT", 0, 3);
//		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
//			LCD.drawString("HMI Mode PAUSE", 0, 3);
//		}else{
//			LCD.drawString("HMI Mode UNKNOWN", 0, 3);
//		}
	}
}
