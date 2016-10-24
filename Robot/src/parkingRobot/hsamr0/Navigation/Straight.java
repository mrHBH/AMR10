package parkingRobot.hsamr0.Navigation;

import lejos.robotics.navigation.Pose;

public abstract class Straight {
	private Straight followingStraight;
	private Pose startPose;
	
	/**
	 * sets both the params for the following straight and the endPose to null; those have to be set later, sets the startPose
	 */
	public Straight(Pose startPose){
		this.startPose = startPose;
		this.followingStraight = null;
	}
	
	/**
	 * sets the following straight if it wasnt set before
	 * @param followingStraight
	 */
	public void setFollowingStraight(Straight followingStraight){
		if(this.followingStraight == null)
			this.followingStraight = followingStraight;
	}
	
	/**
	 * Getter for startPose
	 * @return startPose
	 */
	public Pose getStartPose(){
		return startPose;
	}
	
	/**
	 * Getter for endPose
	 * @return endPose
	 */
	public Pose getEndPose(){
		return followingStraight.getStartPose();
	}
	
	/**
	 * Returns the start point of the straight depending on which axis it is parallel to
	 * @return
	 */
	public float get1DStartPoint(){
		if (this.isYParallel())
			return this.startPose.getY();
		else
			return this.startPose.getX();
	}
	
	/**
	 * Returns the end point of the straight depending on which axis it is parallel to
	 * @return
	 */
	public float get1DEndPoint(){
		if (this.isYParallel())
			return this.getEndPose().getY();
		else
			return this.getEndPose().getX();
	}
	
	/**
	 * returns true if the straight is parallel to the x axis
	 * @return
	 */
	public boolean isXParallel(){
		if(this.getStartPose().getX() == this.getEndPose().getX())
			return true;
		return false;
	}
	
	/**
	 * returns true if the straight is parallel to the y axis
	 * @return
	 */
	public boolean isYParallel(){
		if(this.getStartPose().getY() == this.getEndPose().getY())
			return true;
		return false;
	}

	public abstract boolean hasParkingZone();

}
