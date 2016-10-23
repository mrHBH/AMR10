package parkingRobot.hsamr0.Navigation;

import lejos.robotics.navigation.Pose;

public class NavigationHandler {

	private Straight currentStraight;
	private Pose totalPose;
	
	private Straight initCourse(){
		//init the straights
		StraightParking s1 = new StraightParking(new Pose(0,0,0)); //long straight at the x-axis
		StraightParking s2 = new StraightParking(new Pose(210,0,90)); //middle long straight parallel to y-axis at the very right
		StraightEmpty s3 = new StraightEmpty(new Pose(210,60,180)); //short straight at the top right parallel to x-axis
		StraightEmpty s4 = new StraightEmpty(new Pose(180,60,270)); //short straight downwards at the right parallel to y-axis
		StraightParking s5 = new StraightParking(new Pose(180,30,180)); //middle long straight parallel to x-axis at the top
		StraightEmpty s6 = new StraightEmpty(new Pose(30,30,90)); //short straight at the top left parallel to y-axis
		StraightEmpty s7 = new StraightEmpty(new Pose(30,60,180)); //short straight at the top left parallel to x-axis
		StraightEmpty s8 = new StraightEmpty(new Pose(0,60,270)); //middle long straight at the left parallel to y-axis
		
		//set the followers
		s1.setFollowingStraight(s2);
		s2.setFollowingStraight(s3);
		s3.setFollowingStraight(s4);
		s4.setFollowingStraight(s5);
		s5.setFollowingStraight(s6);
		s6.setFollowingStraight(s7);
		s7.setFollowingStraight(s8);
		s8.setFollowingStraight(s1);
		
		return s1;
	}
	
	public NavigationHandler(){
		currentStraight = this.initCourse();
		this.totalPose = new Pose(0,0,0);
	}
	
	public void setPose(float x, float y, float phi){
		this.totalPose.setLocation(x,y);
		this.totalPose.setHeading(phi);	
		
		boolean turningToNextStraight = (this.currentStraight.getEndPose().getHeading() - this.totalPose.getHeading()) % 360 < 10.0 ;
		boolean turningToParking = ((this.currentStraight.getEndPose().getHeading() - 90) % 360 - this.totalPose.getHeading()) % 360 < 10.0;
		
		if(turningToNextStraight && turningToParking){
			//here fucking straight four turning
		}
		else if(turningToNextStraight){
			//we turn to the next straight; we should calibrate soon
		}
		else if(turningToParking){
			//we turn to parking here; we should switch to the exact pose
		}
	}
	
	public Pose getPose(){
		return this.totalPose;
	}
	
	public Pose getMeasurementPose(){
		return this.totalPose;
	}
}