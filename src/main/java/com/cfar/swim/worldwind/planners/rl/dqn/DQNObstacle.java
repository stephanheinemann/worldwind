package com.cfar.swim.worldwind.planners.rl.dqn;

import java.time.ZonedDateTime;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;

import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;

/**
 * Realizes an obstacle to be used by the DQN planner.
 * 
 * @author Rafaela Seguro
 *
 */
public class DQNObstacle implements Comparable<DQNObstacle>{
	
	/** vector that points from the obstacle center to the state*/
	private Vec4 relativeToState = null;
	
	/** the distance from the obstacle's closest point to the state*/
	private float  distanceToState = 0.0f;
	
	
	/**
	 * Constructs an obstacle instance to be used by a DQN planner. It's characteristics 
	 * are defined relative to a state
	 * 
	 * @param the state's position
	 * @param the obstacle
	 * @param the globe
	 * 
	 */
	public DQNObstacle(Position position, Obstacle obstacle, Globe globe) {
		
		Vec4 statePoint = globe.computePointFromPosition(position);
		Vec4 obstaclePoint = globe.computePointFromPosition(obstacle.getCenter());
		
		this.relativeToState = statePoint.subtract3(obstaclePoint);
		
		this.distanceToState = (float) (this.relativeToState.getLength3() - obstacle.getExtent(globe).getRadius());
	}
	
	
	/**
	 * Gets the vector that points from the obstacle center to the state
	 * 
	 * @return the vector that points from the obstacle center to the state
	 */
	public Vec4 getrelativeToState() {
		return this.relativeToState;
	}
	
	/**
	 * Gets the distance from the obstacle's closest point to the state
	 * 
	 * @return the distance from the obstacle's closest point to the state
	 */
	public double getDistanceToState() {
		return this.distanceToState;
	}
	
	/**
	 * Compares this DQNObstacle to another by their distance to the state
	 * 
	 * @return 0 if the distance is the same; less than 0 if this obstacle is closer to the state; 
	 * greater than 0 if the other obstacle is closer to the state.
	 */
	@Override
	public int compareTo(DQNObstacle obstacle) {
		
		return Double.compare(distanceToState, obstacle.getDistanceToState());
	}

}
