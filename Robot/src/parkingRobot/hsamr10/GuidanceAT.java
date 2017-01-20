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
		 * indicates that robot is following the line and maybe detecting parking slots.
		 */
		SCOUT,
		/**
		 * indicates that robot is following the line to reach the selected slot and will initiate the parking operation there.
		 */
		PARKTHIS,
		/**
		 * indicates that robot is searching for new parking places to park as soon as possible.
		 */
		PARKNOW,
		/**
		 * indicates that robot has parked and is waiting for new input .
		 */
		PARKED,
		/**
		 * This State is only used to ensure compatibility with the HMI module and is not present in this implementation.
		 */
		AUSPARKEN,
		/**
		 * The Robot is idiling and waits for new inputs.
		 */
		INACTIVE,
		/**
		 * The robot will terminate the bluetooth connection and ends the running program.
		 */
		EXIT
		
	}
	/**
	 * States for the Substate machine , they modell the current activity being executed at any moment
	 * @author HBH
	 *
	 */
	public enum CurrentActivity {
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
		 * The robot is in the front  boundary position of the parking slot , will get a parrallel pose and park in . 
		 */
		PARKING, 
		/**
		 * The robot follows the line autonomously.
		 */
		LINEFOLLOWING, 
		/**
		 * The Robot is excecuting the Parking out algotrithm .
		 */
		
		PARKINGOUT
	}
	
	
	
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.INACTIVE;
	/**
	 * state in which the sub-state machine is running at the moment
	 */
	protected static CurrentActivity currentActivity = CurrentActivity.NOSUBSTATE;
	/**
	 * 	protected static CurrentStatus lastStatus		= CurrentStatus.INACTIVE;

	 */
	protected static CurrentActivity lastActivity = CurrentActivity.NOSUBSTATE;
    protected static CurrentStatus gotStatus= CurrentStatus.INACTIVE;

	
	/**
	 * one line of the map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * This documentation for line0 hold for all lines.
	 * Added  the Heading corresponding to every Line
	 * 
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
	static int slotHeading =0;

      static 	int lastid=0;

	
	static Point  origin=new Point(0.0f,0.0f) ;
    static Point  destination=new Point(0f,0f) ;

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
        lastActivity =CurrentActivity.NOSUBSTATE;		 	
        currentActivity=CurrentActivity.NOSUBSTATE;
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
		//monitor.startLogging();
		
		
		/** Defined Variables */
		
		/** Numericals :  */
		
		/**
		 * Collision threshold is a variable that sets a threshold distance. If the front IR sensors detects a value lower than the 
		 * threshold , a collision is then detected.
		 */
		double CollisionThreshod=0;
	
		
		int currentHeading=0;
        int x;
        /** Booleans */
        
		/**The booleans defined here are used for transistioning to the suitable sub-state 
		
		 */
		/**
		 * Collision Detected is set to true when the front IR sensor returns a value lower than the defined Threshold
		 */
		boolean collisionDetected=false;
		/**
		 * If the Robot has found a potential parking slot , 
		 * -implemented as the side sensors returned a defined amount of consequitive inifiniy values -
		 * this Variable is set to true
		 */
		
		boolean foundPotentialSlot=false;
		/**
		 * If the robot has reached the Defined destination , this variable is set to true
		 * The robot reaches the defined destination if it is within a defined distance from the destination point
		 */
		boolean reacheddestination=false;
		/**
		 * If the last Parking Slot is suitable for Parking , this variable is set to true 
		 * This variable is important for parking now mode.
		 */
	    boolean lastIsSuitable=false;
       
	    /**
	     * These two booleans describe if the robot successfylly entered / left a parking slot
	     */
	    boolean successfullyParkedOut=false;
	    boolean successfullyParkedIn=false;
	    boolean terminatedParking=false;
	    boolean goToInactive,goToScout,goToParked,goToParknow,goToParkthis,goToExit=false;
	    /**Points */
	    Point  destination=new Point(0f,0f) ;
	    Point  destinationm=new Point(0f,0f) ;

	
	    /** Other Data Types */
		ParkingSlot[] Array ;
		Mode gotTabletInput=Mode.PAUSE;
		//int NumberofSuitable=0;
		
	
	    /** 
	     * Main Program loop 
	     * In this loop the program updates the variables needed for The main and the sub-State machines 
	     * it then switches to the suitable state or substate 
	     * This part of the program can be further  divided to two  areas : 
	     * Update() Area 
	     * Excexute() Area 
	     */
	    
		while(true) {
			/**
			 * Update Area
			 * Tertiary operators are used to make the code easier to read and more compacter 
			 */
			
			/**
			 * Updates the parking slot array 
			 */
			
			Array=navigation.getParkingSlots();
			
			
			/**
			 * if the front IR sensors retunrs a value lower than the threshold collision detected is true 
			 * else it is set to false
			 */
			collisionDetected = perception.getFrontSensorDistance()<CollisionThreshod ? true : false;
		    foundPotentialSlot = navigation.detectingParkingSlot() ? true : false;
			//foundPotentialSlot=perception.getFrontSensorDistance()==perception. ? true : false;
		    /**
		     * if the robot is within the defined from the distination , this variable is set to true
		     * the defined distance of 0.1177157 was determined experimentally to allow for a safer parking manoever
		     */
			reacheddestination = (navigation.getPose().distanceTo(destinationm)<0.1177157) ? true : false;
			
			/** 
			 * CurrentHeading is the heading the robot should have when on a specific line 
			 * Implementation : if the current line is 0 , then the mustHeading is 0 , if current line is 1 then must heading 90 and so on 
			 * x is the line number on which the robot is currently driving
			 */
			lastid=navigation.getLastChangedSlot();
	
			x=navigation.getLineID();
			currentHeading=x==0?heading0:x==1?heading1:x==2?heading2:x==3?heading3:x==4?heading4:x==5?heading5:x==6?heading6:heading7;
			/**
			 *  In control module two booleans with the same meaning were already implemented 
			 *  I simply transfer them to the booleans defined in guidance to make code simpler
			 */
			successfullyParkedOut=control.succesfullyParkedOut;
			successfullyParkedIn=control.succesfullyParkedIn;
		
			/**
			 * In this section the variable needed for transition check are updated
			 * goToMODENAME is a boolean that when true the robot will go to that specific mode
			 */
			gotTabletInput = hmi.getMode();
		    goToInactive = gotTabletInput == parkingRobot.INxtHmi.Mode.PAUSE ? true : false;
		    goToScout = gotTabletInput == parkingRobot.INxtHmi.Mode.SCOUT ? true : false;
		//    goToParked = gotTabletInput == parkingRobot.INxtHmi.Mode. ? true : false;
		    goToParknow = gotTabletInput == parkingRobot.INxtHmi.Mode.PARK_NOW ? true : false;
		    goToParkthis = gotTabletInput == parkingRobot.INxtHmi.Mode.PARK_THIS? true : false;
			        if( gotTabletInput == parkingRobot.INxtHmi.Mode.PAUSE ){
						gotStatus = CurrentStatus.INACTIVE;	
			     	    }else if (gotTabletInput == parkingRobot.INxtHmi.Mode.DISCONNECT){
			     		gotStatus = CurrentStatus.EXIT;
			     		}else if ( gotTabletInput == parkingRobot.INxtHmi.Mode.PARK_THIS ){
						gotStatus = CurrentStatus.PARKTHIS;
						}else if ( gotTabletInput == parkingRobot.INxtHmi.Mode.SCOUT ){
				    	gotStatus = CurrentStatus.SCOUT;
				    	}else if ( gotTabletInput == parkingRobot.INxtHmi.Mode.PARK_NOW ){
				    	gotStatus = CurrentStatus.PARKNOW;}
			        
			showData(navigation, perception);

			/** Excecute Area */
			/**
			 * Each FSM is modelled with the switch case structure 
			 * The main state machine models the different modes the robot can be in 
			 * The sub-state machine models the different operations needed to perform the task 
			 * The robot can only switch to a different main state , if is not performing any activity implemented as the NOSUBSTATE sub-state
			 * this was implemented to avoid not allowed states , like parking and scouting in the same time . 
			 * 
			 */
			
		    
			/**
			 * This first condition is the implementation of the separation between the two state machines 
			 * the robot can only switch to a different state if the current substate is NOSUBSTATE
			 * The state 'exit' is an exception to this rule and the robot should switch immediately to this state when asked to 
			 */
			 
			if ( ( currentActivity==CurrentActivity.NOSUBSTATE) || (currentStatus==CurrentStatus.EXIT) )
			
			{
				
        	switch ( currentStatus )
        	{  
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------*/        	
        	case PARKNOW:
        	  
//Into action
        		if ( lastStatus != CurrentStatus.PARKNOW )
				{
        			/**
        			 * Activates the forward line following
        			 */
                	navigation.setLineFollowerState(true);
                    lastIsSuitable=false;
					control.setBackwords(false);
					control.setCtrlMode(ControlMode.LINE_CTRL);
					/**
					 * Defines the Collision Threshold 
					 * was experimentally determined to suit the geometry of the map
					 */
                    CollisionThreshod=7;
                    terminatedParking=false;
                    /**
                     * Activates detection of Parking slots
                     */
					navigation.setDetectionState(true);
				}
					
			
//While action		
        		
				{
					//nothing to do here
				}
				
//State transition check
				
		      	lastStatus = currentStatus;
                if ( Button.ENTER.isDown() ){
					currentStatus = CurrentStatus.INACTIVE;
					while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
                }else if ( Button.LEFT.isDown() ){
					currentStatus = CurrentStatus.PARKNOW;
					while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
				}else if ( Button.ESCAPE.isDown() ){
					currentStatus = CurrentStatus.EXIT;
					while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
			    }
				else if (goToExit||goToInactive||goToScout||goToParkthis){
					currentStatus = gotStatus;
                }
				else if (terminatedParking) // the robot finished parking succsessfully
					currentStatus=CurrentStatus.PARKED;
                
                
//Susbtate Transition check
                
               /** 
                * Collision Detection
                * if collisionDetected is set to true in the main loop , the robot will go to avoiding collision Substate 
                */
                if (collisionDetected){
					currentActivity=CurrentActivity.AVOIDINGCOLLISION; 
				}
               
			   /**
			    * If the robot just measured a suitable parking slot this  variable is set to true (in measuring substate ) and the robot goes 
			    * to parking susbtate 
			    */
			    else if (lastIsSuitable)
				{  
					control.setCtrlMode(ControlMode.INACTIVE);
                    currentActivity=CurrentActivity.PARKING;

				}
                /**
                 * if this boolean is set to true in the main loop the robot goes to measuring substate
                 */
			    else if ( foundPotentialSlot){
					currentActivity=CurrentActivity.MEASURING;
				}
                
                
		
//Leave action
			    else if ( currentStatus != CurrentStatus.PARKNOW ){
					//nothing to do here
				}
               
				break;			
        		
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        	
        
        	case PARKED:
//Into action 
        		if ( lastStatus != CurrentStatus.PARKED )
				{
                	navigation.setLineFollowerState(false);
                	successfullyParkedOut=false;
                	lastStatus=currentStatus;

                }
//while action        		
        		{
        			
        		}
//State Transition check
        		
        		if (successfullyParkedOut)
        			{
        			currentStatus=CurrentStatus.SCOUT;
        			}
        		
        	
//Susbtate Transition check 
        		if ( gotStatus==CurrentStatus.SCOUT ){
					currentActivity=CurrentActivity.PARKINGOUT;		
					}
        		else if ( Button.ENTER.isDown()){
					currentActivity=CurrentActivity.PARKINGOUT;		
					while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
        		}
//Leave action        		
        		
        		if ( currentStatus != CurrentStatus.PARKED ){
					//nothing to do here
				}

        	break;
        	
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/        	
				case SCOUT:
				
// Into action
					
					if ( lastStatus != CurrentStatus.SCOUT // lastActivity !=CurrentActivity.NOSUBSTATE // currentActivity=CurrentActivity.LINEFOLLOWING;
					){
						control.setBackwords(false);
						control.setCtrlMode(ControlMode.LINE_CTRL);
                        CollisionThreshod=7;
						navigation.setDetectionState(true);
	                	navigation.setLineFollowerState(true);

					}
					
// While action				
					
					{
						//nothing to do here
					}					
					
					
// State transition check
					lastStatus = currentStatus;
			
					if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.INACTIVE;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}
					else if (goToExit||goToInactive||goToParknow||goToParkthis){
						currentStatus=gotStatus;
			     	}

					
				   
// SubState Transition check
					
				   if (collisionDetected){ 				  // Collision detected  and going to the avoiding collision substate
                      	currentActivity=CurrentActivity.AVOIDINGCOLLISION; 
					}else if ( foundPotentialSlot ){          //  The robot detected a parking and will go to measuring substate   
                        currentActivity=CurrentActivity.MEASURING;
					}
					
				  // Leave action
					if ( currentStatus != CurrentStatus.SCOUT ){
						//nothing to do here
					}
					break;				
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/			
				case INACTIVE:
//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
	                	navigation.setLineFollowerState(false);

						
					}
					
//While action
					{
						//nothing to do here
					}
					
//State transition check
					lastStatus = currentStatus;
				
				    if ( Button.ENTER.isDown() ){
						currentStatus = CurrentStatus.SCOUT;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.LEFT.isDown() ){
						currentStatus = CurrentStatus.PARKNOW;
						while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
					}else if ( Button.ESCAPE.isDown() ){
						currentStatus = CurrentStatus.EXIT;
						while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
					}else if (goToExit||goToParknow||goToScout){
							currentStatus=gotStatus;
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
				
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------*/				
				case PARKTHIS:
 //Into action
		        		
						if ( lastStatus != CurrentStatus.PARKTHIS  )
						{
							lastStatus = currentStatus;

							control.setBackwords(false);
							control.setCtrlMode(ControlMode.LINE_CTRL);
		                	navigation.setLineFollowerState(true);

		                    CollisionThreshod=7;
							navigation.setDetectionState(false);
							destination=Array[hmi.getSelectedParkingSlot()].getFrontBoundaryPosition(); // cm
		        			destinationm.x=(float) (destination.getX()/100);
		        			destinationm.y=(float) (destination.getY()/100);
		        			reacheddestination=false;
		        			terminatedParking=false;

			            }
//While action				
						{
							//nothing to do here
						}					
						
						
//State transition check
						
					
					    if ( Button.ENTER.isDown() ){
							currentStatus = CurrentStatus.INACTIVE;
							while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
						}else if ( Button.ESCAPE.isDown() ){
							currentStatus = CurrentStatus.EXIT;
							while(Button.ESCAPE.isDown()){Thread.sleep(1);}
							//wait for button release
						
						}else if (goToExit||goToInactive||goToParknow||goToScout){
							currentStatus=gotStatus;
				     	}
						
						else if (terminatedParking) // the robot finished parking succsessfully
							currentStatus=CurrentStatus.PARKED;
					    
		                
						
						
// Activity Launcher
						if (collisionDetected){
							currentActivity=CurrentActivity.AVOIDINGCOLLISION; 
						  }
					    else if (reacheddestination)
					    {
							currentActivity=CurrentActivity.PARKING; 
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
/**
 * The second switch case structure models the nested state machine 
 * 			
 */
			
			// The substate machine , models the current activity of the robot ; 
			 switch ( currentActivity )
			  
        	{ 
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------*/                	

                case PARKING:
//Into action  
                if (lastActivity!=currentActivity)
                {
                navigation.setLineFollowerState(false);
                terminatedParking=false;
        		lastActivity=currentActivity;
			    control.destination.setHeading( (float) (slotHeading*Math.PI/180));
          /* while (navigation.getPose().getHeading()!=slotHeading*Math.PI/180)
			      {
				  control.turnDefinedAngle();
		          }*/
        		}
        		
  		
//While action        		 
                
                /**
                 *The robots alligns its pose to be parrallel to the parking slot
                 *it thens drives backward a distance of 0.1175 backwards from frontboundary 
                 *after that it executes the parking polynom
                 *once finished it drives backwards till the end of the backboundary of the slot
                 */
                
           /*     if (navigation.getPose().getHeading()!=currentHeading*Math.PI/180)
			      {
				  control.destination.setHeading( (float) (currentHeading*Math.PI/180));
				  control.turnDefinedAngle();
		          }*/
                else  if (navigation.getPose().distanceTo(origin)<0.0475){
        	    	    control.drive(-5, 0); }
        		else if (navigation.getPose().distanceTo(origin)>0.0475){
          				control.setCtrlMode(ControlMode.PARK_CTRL);}
				 if (successfullyParkedIn) 
		        {control.setCtrlMode(ControlMode.INACTIVE);
				         if( perception.getBackSensorDistance()>2)  
        			     {
        				 control.drive(-5, 0);
        			     }
        			     else
        			     {
        				 control.setCtrlMode(ControlMode.INACTIVE);
        		         currentActivity=CurrentActivity.NOSUBSTATE;
        		         }
        	   }
                
//Leave action 
                if ( currentActivity != CurrentActivity.PARKING  )
                {
                 terminatedParking = true;
                 reacheddestination=false;
                }
                        
        	    break;
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/        			 
        	case MEASURING:
        	
        		 
//Entry Action
               
    		if (lastActivity!=currentActivity){
    			lastActivity=currentActivity;
    			lastIsSuitable=false; // resets the boolean every time a new slot is being measured
    		}
//While action 
    		

    		 if (currentStatus == CurrentStatus.EXIT ) 
    		    {
    			currentActivity=CurrentActivity.NOSUBSTATE;
    			break;
    			} 
    		else if (collisionDetected){currentActivity=CurrentActivity.AVOIDINGCOLLISION;}
    	
                   //Leave Action 
    		else  if (  !navigation.detectingParkingSlot() ) {
   			
    		   origin.setLocation(navigation.getPose().getX(), navigation.getPose().getY());;
    		   
    		   Array= navigation.getParkingSlots();
    	
    		//   if (Array.length!=0){  lastIsSuitable = Array[Array.length-1].getStatus()==ParkingSlotStatus.SUITABLE_FOR_PARKING ? true : false;}
		           // lastIsSuitable = Array[Array.length-1].getStatus()==ParkingSlotStatus.SUITABLE_FOR_PARKING ? true : false;
    		   if (Array.length!=0 && navigation.getLastChangedSlot()<=Array.length) { lastIsSuitable = Array[navigation.getLastChangedSlot()].getStatus()==ParkingSlotStatus.SUITABLE_FOR_PARKING ? true : false;}
                if (lastIsSuitable) {lejos.nxt.Sound.twoBeeps();

               
                
                
                }
    			slotHeading=currentHeading;

          		//currentStatus=CurrentStatus.INACTIVE;

          		currentActivity=CurrentActivity.NOSUBSTATE;
                }

// Leave action 
    		 
    		 break;
			  
        	
/*------------------------------------------------------------------------------------------------------------------------------------------------------------*/       	
        	case AVOIDINGCOLLISION:
        		
        		/** Avoiding Collision 
        		 * the Robot stops when an obstacle within "CollisionThreshold" is detected . 
        		 * if the obstacle approaches beyond the distance 2 from the front , The robot will step backwards
        		 * if the obstacle is static and still within the  "collision threshold" the robots stops and waits 
        		 * if the collision is cleared , the robot leaves "avoiding Collision" to "NOSUBSTATE" 
          		 */
// Into action
        		if (lastActivity!=currentActivity){

        			control.setCtrlMode(ControlMode.INACTIVE);
    				lastActivity=currentActivity;
        			}
// While action
        		if (currentStatus == CurrentStatus.EXIT ) {  // If the user chooses to leave the program exits immediately 
        			currentActivity=CurrentActivity.NOSUBSTATE;
        		    break;
        		}
        		
                if (perception.getFrontSensorDistance()>=2 && perception.getFrontSensorDistance()<CollisionThreshod){
             		control.setCtrlMode(ControlMode.INACTIVE);
             	}
            
                
             	else if (perception.getFrontSensorDistance()<2) {  
             		
             	// The Collision is approaching : step backwards
             if (perception.getBackSensorDistance()>3){
             		control.drive(-5,0);
             }
             	}
            	 
             	else {            
             		// The Collision is cleared : exit the substate 
                    collisionDetected=false;
           		    control.setBackwords(false); // this disables the backward line follower 
				    control.setCtrlMode(ControlMode.LINE_CTRL);
                    currentActivity=CurrentActivity.NOSUBSTATE;}
//Leave action
                if (currentActivity!=CurrentActivity.AVOIDINGCOLLISION)
                {

                }
                
				break;
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/				
				/**
				 * In this substate the robot simply idles and does nothing
				 */
        	case NOSUBSTATE : 
					
//Into action
					{
	        			lastActivity=currentActivity;

					}
//while action  
					{
					}
//Leave action      
					{
					}
					
					break;
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/					
				case LINEFOLLOWING:
//into action
				if (lastActivity!=currentActivity){
					lastActivity=currentActivity;
					control.setCtrlMode(ControlMode.LINE_CTRL);
					navigation.setLineFollowerState(true);
					}
//while action
				{
					
				}
				
//leave action
				if (lastActivity!=CurrentActivity.LINEFOLLOWING){
                    control.setCtrlMode(ControlMode.INACTIVE);
				}
					
				break;
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/				
				case PARKINGOUT:        

					
//Into action
	        		if (lastActivity!=currentActivity)
	        		{
	        			lastActivity=currentActivity;
	                	navigation.setLineFollowerState(false);

	        		}
//while action
	        		{
	        			
	        		}
					
					control.setCtrlMode(ControlMode.PARK_CTRL);
					if (successfullyParkedOut) {
						currentActivity=CurrentActivity.NOSUBSTATE;
					}
					
//leave action
					
					if (lastActivity!=CurrentActivity.PARKINGOUT)
	        		{
						currentStatus=CurrentStatus.SCOUT;
	                	navigation.setLineFollowerState(true);

	        		}
					
			    	default:
	        		break;
        		}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------*/
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
	    LCD.drawString("Front: "+perception.getFrontSensorDistance(), 0, 2);	
        LCD.drawString("STATE: "+currentStatus, 0, 0);
		LCD.drawString("SUBSTATE: "+currentActivity, 0, 1);
        LCD.drawString("DfroBO: "+navigation.getPose().distanceTo(origin), 0, 4);	
    //    LCD.drawString("dtoorigin: "+navigation.getPose().distanceTo(destination), 0, 5);	
           LCD.drawString("slotH: "+slotHeading, 0, 5);	
           LCD.drawString("lastID: "+lastid, 0, 6);	


	}
}
