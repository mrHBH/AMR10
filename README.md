# AMR10
AMR



public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and detecting parking
		 * slots
		 */
		SCOUT,
		/**
		 * indicates that robot is following the line to a special parking slot
		 * (ID) and park into this slot
		 */
		PARK_THIS,
		/**
		 * indicates that robot is following the line to the next parking slot
		 * and park into this slot
		 */
		PARK_NOW,
		/**
		 * indicates that robot is performing an parking maneuver
		 */
		INACTIVE,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT
	}
