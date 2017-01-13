package parkingRobot.hsamr10;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.robotics.navigation.DestinationUnreachableException;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.hsamr10.ControlRST;
import parkingRobot.hsamr10.HmiPLT;
import parkingRobot.hsamr10.NavigationAT;
import parkingRobot.hsamr10.PerceptionPMP;
import parkingRobot.INxtHmi;
import parkingRobot.INxtHmi.Mode;
import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import javax.microedition.sensor.MeasurementRange;

import lejos.geom.Line;
import lejos.geom.Point;
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
		 * indicates that robot is following the line to reach the selected slot and will initiate the parking operation there.
		 */
		PARKTHIS,
		/**
		 * indicates that robot is searching for new parking places to park as soon as possible
		 */
		PARKNOW,
		/**
		 * indicates that robot is idling
		 */
		PARKED,
		AUSPARKEN,
		INACTIVE,
		
		
		/**
		 * indicates the Robot is performing the leaving maneuver and  will go to scout mode upon termination
		 */
		//AUSPARKEN,
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
		NOSUBSTATE,
		/** 
		 * The robot is in the back boundary position of the parking slot , have a parallel pose and ready to start the parking operation 
		 */
		PARKING, 
		LINEFOLLOWING, PARKINGOUT
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
	static int  heading0=0;
	static Line line1 = new Line(180,  0, 180, 60);
	static int  heading1=90;
	static Line line2 = new Line(180, 60, 150, 60);
	static int  heading2=180;
    static Line line3 = new Line(150, 60, 150, 30);
	static int  heading3=270;
    static Line line4 = new Line(150, 30,  30, 30);
	static int  heading4=180;
    static Line line5 = new Line( 30, 30,  30, 60);
	static int  heading5=90;
	static Line line6 = new Line( 30, 60,   0, 60);
	static int  heading6=180;
	static Line line7 = new Line(  0, 60,   0,  0);
	static int  heading7=270;
	static Point  Origin=new Point(0.0f,0.0f) ;
    static Point  Destination=new Point(0f,0f) ;

	/**
	 * map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * All above defined lines are bundled in this array and to form the course map.
	 */
	static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};
//	private static int NumberofSuitable=0;
	//private static boolean PARKNOW;
	
	
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
		
		NavigationAT navigation = new NavigationAT(perception, monitor);
		ControlRST    control    = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
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
		boolean ReachedDestination=false;
	     boolean LastisSuitable=false;
        int currentHeading=0;
        int x;
		ParkingSlot[] Array ;
		//int NumberofSuitable=0;
		
	    Point  Destination=new Point(0f,0f) ;
	    Point  Destinationm=new Point(0f,0f) ;
	//   Point  Origin=new Point(0.0f,0.0f) ;


		//Destination.x=0;
		//Destination.y=0;
		while(true) {
			 x=navigation.getLineID();
			showData(navigation, perception);
			 Array=navigation.getParkingSlots();
			 CollisionDetected = perception.getFrontSensorDistance()<CollisionThreshod ? true : false;
			 FoundPotentialSlot = navigation.detectingParkingSlot() ? true : false;
			 ReachedDestination = (navigation.getPose().distanceTo(Destinationm)<0.1177157) ? true : false;
			 
			 currentHeading=x==0?heading0:x==1?heading1:x==2?heading2:x==3?heading3:x==4?heading4:x==5?heading5:x==6?heading6:heading7;
			/* if (Array.length!=0) {
			    LCD.drawString("Distan to B: "+navigation.getPose().distanceTo(Destinationm), 0, 4);	
				LCD.drawString("BackbP_X: "+Array[0].getBackBoundaryPosition().getX(), 0, 6);	
				LCD.drawString("BackbP_Y: "+Array[0].getBackBoundaryPosition().getY(), 0, 5);	
			 }*/
			//The RObot can only go to one of the main states if it is not currently inside any substate
			if ( ( currentSubState==CurrentSubState.NOSUBSTATE) || (currentStatus==CurrentStatus.EXIT) /*|| (currentStatus!=CurrentStatus.INACTIVE)*/)
			
			{
				
        	switch ( currentStatus )
        	{   case PARKNOW:
        	 //Into action
        		if ( lastStatus != CurrentStatus.PARKNOW ) // || lastSubState !=CurrentSubState.NOSUBSTATE )
				{
					control.setBackwords(false);
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
				else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS ){
					currentStatus = CurrentStatus.PARKTHIS;}
			    else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
						currentStatus = CurrentStatus.SCOUT;
		     	}
			      	
				
				
				
				//if collisiondetected is set to true in the main loop , the robot will go to avoiding collision Substate 
				else if (CollisionDetected){
					currentSubState=CurrentSubState.AVOIDINGCOLLISION; 
				}
			    
			    // Collision detected  and going to the avoiding collision substate
			    else if (LostTrack) { 
					currentSubState=CurrentSubState.FINDINGTRACK; } // Lost The line and the robot is trying to find it 
			    else if (LastisSuitable)
				{  //  currentSubState=CurrentSubState.PARKING; 
					//navigation.setDetectionState(false);
					control.setCtrlMode(ControlMode.INACTIVE);
                    currentSubState=CurrentSubState.PARKING;

				}
			    else if ( FoundPotentialSlot){
					currentSubState=CurrentSubState.MEASURING;
				}
			  /*  else if (navigation.getPose().distanceTo(Destination)<2)
			    {
					control.setCtrlMode(ControlMode.INACTIVE );
 }
	*/			
				//Leave action
			    else	if ( currentStatus != CurrentStatus.PARKNOW ){
					//nothing to do here
				}
				break;			
        		
        	
        	case PARKED:
        		
        		if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
					currentSubState=CurrentSubState.PARKINGOUT;		
					}
        		else if ( Button.ENTER.isDown() ){
					currentSubState=CurrentSubState.PARKINGOUT;		
					while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
        		}
        	break;
        	
				case SCOUT:
				
				// Into action
					
					if ( lastStatus != CurrentStatus.SCOUT // lastSubState !=CurrentSubState.NOSUBSTATE // currentSubState=CurrentSubState.LINEFOLLOWING;
					){
						control.setBackwords(false);
						control.setCtrlMode(ControlMode.LINE_CTRL);
                        CollisionThreshod=7;
						navigation.setDetectionState(true);
					}
					
				// While action				
					
					{
						//nothing to do here
					}					
					
					
				// State transition check
					lastStatus = currentStatus;
					if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
						currentStatus = CurrentStatus.INACTIVE;}
				    else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW ){
							currentStatus = CurrentStatus.PARKNOW;}
					else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS ){
								currentStatus = CurrentStatus.PARKTHIS;
					}else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
						currentStatus = CurrentStatus.EXIT;
					}
					
				   
				 // SubState Transition check
					
					if (CollisionDetected){ 				  // Collision detected  and going to the avoiding collision substate
                      	currentSubState=CurrentSubState.AVOIDINGCOLLISION; 
					}else if ( FoundPotentialSlot ){          //  The robot detected a parking and will go to measuring substate   
                        currentSubState=CurrentSubState.MEASURING;
					}
					
				  // Leave action
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
					}
				    else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW ){
						currentStatus = CurrentStatus.PARKNOW;}
				    else if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.PARKNOW;
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
					  
		        	
		        	
		        		 //Into action
		        		
						if ( lastStatus != CurrentStatus.PARKTHIS || lastSubState !=CurrentSubState.NOSUBSTATE )
						{
							control.setBackwords(false);
							control.setCtrlMode(ControlMode.LINE_CTRL);
		                    CollisionThreshod=7;
							navigation.setDetectionState(false);
							Destination=Array[hmi.getSelectedParkingSlot()].getFrontBoundaryPosition(); // cm
		        			Destinationm.x=(float) (Destination.getX()/100);
		        			Destinationm.y=(float) (Destination.getY()/100);

		        			

			            }
						//While action				
						{
							//nothing to do here
						}					
						
						
						//State transition check
						
						lastStatus = currentStatus;
						if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
							currentStatus = CurrentStatus.INACTIVE;}
					    else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW ){
								currentStatus = CurrentStatus.PARKNOW;}
						else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
									currentStatus = CurrentStatus.SCOUT;
						}else if ( Button.ENTER.isDown() ){
							currentStatus = CurrentStatus.INACTIVE;
							while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
						}else if ( Button.ESCAPE.isDown() ){
							currentStatus = CurrentStatus.EXIT;
							while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
						}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
							currentStatus = CurrentStatus.EXIT;
						}
						
						
						//SubState transition check
						
						//if collisiondetected is set to true in the main loop , the robot will go to avoiding collision Substate 
						 if  (CollisionDetected){
							currentSubState=CurrentSubState.AVOIDINGCOLLISION; 
						  }
					    // Collision detected  and going to the avoiding collision substate
					    else if (LostTrack) { 
							currentSubState=CurrentSubState.FINDINGTRACK; 
						  }
						 // Lost The line and the robot is trying to find it 
					    else if (ReachedDestination)
					    {
							currentSubState=CurrentSubState.PARKING; 

						//currentSubState=CurrentSubState.PARKING;
                        }
						
						 
						//Leave action
						if ( currentStatus != CurrentStatus.PARKTHIS ){
						//nothing to do here
						}
						break;			
		        		
					
					
			 default:
				break;
        	
			}
        	
        }
			
			// The substate machine , models the current activity of the robot ; 
			 switch ( currentSubState )
			  
        	{ case FINDINGTRACK:
        		break;
        	  case PARKING:
        		  if (lastSubState!=currentSubState){
        			  lastSubState=currentSubState;
        			if (navigation.getPose().getHeading()!=currentHeading*Math.PI/180){
        				
        				  control.destination.setHeading( (float) (currentHeading*Math.PI/180));
        				  control.turnDefinedAngle();
        			  }
        			  
        		  }
  				//control.setCtrlMode(ControlMode.INACTIVE);
        //		lastSubState=currentSubState;
        	//if (navigation.getPose().distanceTo(Origin)>0.1177175)
        		  
        		 
        		else if  (navigation.getPose().distanceTo(Origin)<0.1175)
             	    { control.drive(-5, 0); }
                //	else if ( navigation.getPose().distanceTo(Array[Array.length-1].getBackBoundaryPosition()) < 0.1177157) {
        		else if (navigation.getPose().distanceTo(Origin)>0.1175){
        			//control.setCtrlMode(ControlMode.INACTIVE);}
				control.setCtrlMode(ControlMode.PARK_CTRL);}
				
        	
        	    
        		 if (control.succesfullyParkedIn) {
        			 if( perception.getBackSensorDistance()>2)
        			 {
        				 control.drive(-5, 0);
        			 }
        			 else {
        				 control.setCtrlMode(ControlMode.INACTIVE);
        			 
        		
        			 currentStatus=CurrentStatus.PARKED;
        		 currentSubState=CurrentSubState.NOSUBSTATE;
        		 }}
        			 break;
        			 
        	case MEASURING:
        	
        		 
        		   //Entry Action
               
    		if (lastSubState!=currentSubState){
    			lastSubState=currentSubState;
    			LastisSuitable=false;
    		}

    		else if (currentStatus == CurrentStatus.EXIT ) {
    			currentSubState=CurrentSubState.NOSUBSTATE;
    			break;} 
    		else if (CollisionDetected){currentSubState=CurrentSubState.AVOIDINGCOLLISION;}
    	
                   //Leave Action 
    		else  if (  !navigation.detectingParkingSlot() ) {
   			 //   Array=navigation.getParkingSlots();
    		//	Origin=navigation.getPose().getLocation();
    		    Origin.setLocation(navigation.getPose().getX(), navigation.getPose().getY());;
    	//		Origin.y/=100;

    //		Origin=( Array[Array.length-1].getBackBoundaryPosition());
    	//	Origin.x/=100;
    		//Origin.y/=100;
    		//	Origin.setLocation(Origin.getX()/100, Origin.getY()/100);
    		   if (Array.length!=0){ LastisSuitable = Array[Array.length-1].getStatus()==ParkingSlotStatus.SUITABLE_FOR_PARKING ? true : false;}
		      //LastisSuitable = Array[navigation.getLastChangedSlot()].getStatus()==ParkingSlotStatus.SUITABLE_FOR_PARKING ? true : false;

                if (LastisSuitable==true) {lejos.nxt.Sound.beepSequence();
          		currentSubState=CurrentSubState.NOSUBSTATE;
          	//	currentStatus=CurrentStatus.INACTIVE;

                }
          		//PARKNOW = Array[Array.length-1].getStatus()==ParkingSlotStatus.SUITABLE_FOR_PARKING ? true : false;
          		currentSubState=CurrentSubState.NOSUBSTATE;

    		}
             
        		break;
			  
        	
        	
        	case AVOIDINGCOLLISION:
        		
        		/** Avoiding Collision 
        		 * the Robot stops when an obstacle within "CollisionThreshold" is detected . 
        		 * if the obstacle approaches beyond the distance 2 from the front , The robot will step backwards
        		 * if the obstacle is static and still within the  "collision threshold" the robots stops and waits 
        		 * if the collision is cleared , the robot leaves "avoiding Collision" to "NOSUBSTATE" 
          		 */
        		

				// Into action
        		if (lastSubState!=currentSubState){
        			control.setCtrlMode(ControlMode.INACTIVE);
    				lastSubState=currentSubState;
        			}
        		// While action
        		{
        			currentDistance=perception.getFrontSensorDistance();   // Updates the Front Sensor Distance
        		}
        		//  Main State Transition Check 
        		
        		if (currentStatus == CurrentStatus.EXIT ) {  // If the user chooses to leave the program exits immediately 
        			currentSubState=CurrentSubState.NOSUBSTATE;
        		    break;
        		}
        		
        	   // 
                 	if (currentDistance>=2 && currentDistance<CollisionThreshod){
             		control.setCtrlMode(ControlMode.INACTIVE);
             	}
            	
             	else if ((currentDistance<2)&&(perception.getBackSensorDistance()>5)) {  // The Collision is approaching : step backwards
             	control.drive(-5,0);
          		// control.setBackwords(true); // this enables the backward line follower
			     //control.setCtrlMode(ControlMode.LINE_CTRL);
             	
             		
             	}
            	 
             	else {            
             		// The Collision is cleared : exit the substate 
                    CollisionDetected=false;
           		    control.setBackwords(false); // this disables the backward line follower 
				    control.setCtrlMode(ControlMode.LINE_CTRL);
                    currentSubState=CurrentSubState.NOSUBSTATE;}
             
				break;
				
				case NOSUBSTATE : 
					//lastSubState=currentSubState;
					break;
					
				case LINEFOLLOWING:
					
				break;
				case PARKINGOUT:
					//control.parking=false;
					control.setCtrlMode(ControlMode.PARK_CTRL);
					if (control.succesfullyParkedOut) {
						currentStatus=CurrentStatus.SCOUT;
						currentSubState=CurrentSubState.NOSUBSTATE;
						
					}
				default:
	        		break;
        		}
			 
/*
		    if (perception.getFrontSensorDistance()<CollisionThreshod){CollisionDetected=true;}
		    if (navigation.detectingParkingSlot()){FoundPotentialSlot=true;}
		    if (navigation.getPose().distanceTo(Destination)<2&& ( Destination.x!=0 && Destination.y!=0)){ReachedDestination=true;}
			     	Thread.sleep(10);      
			     	*/  	
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
	//	LCD.drawString("Backside: "+perception.getBackSideSensorDistance(), 0, 5);	
		//LCD.drawString("frontside: "+perception.getFrontSideSensorDistance(), 0, 6);	
	  //  Point  origin=new Point(0f,0f) ;
	//	LCD.drawString("D to 0: "+navigation.getPose().distanceTo(origin), 0, 6);	

		LCD.drawString("Front: "+perception.getFrontSensorDistance(), 0, 2);	
	//	LCD.drawString("PARKNOW: "+PARKNOW, 0, 3);	

		//LCD.drawString("Back: "+perception.getBackSensorDistance(), 0, 3);	
	//	LCD.drawString("PARKNOW: "+ GuidanceAT.PARKNOW , 0, 5);	

	//	LCD.drawString("SLOTS: "+Array.length, 0, 3);	
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
		LCD.drawString("STATE: "+currentStatus, 0, 0);
		LCD.drawString("SUBSTATE: "+currentSubState, 0, 1);
	//	LCD.drawString("PArking?"+PARKNOW, 0, 1);
        LCD.drawString("DfroBO: "+navigation.getPose().distanceTo(Origin), 0, 4);	
        LCD.drawString("dtoorigin: "+navigation.getPose().distanceTo(Destination), 0, 5);	

		
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
