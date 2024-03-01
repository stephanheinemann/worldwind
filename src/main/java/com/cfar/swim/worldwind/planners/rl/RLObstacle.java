package com.cfar.swim.worldwind.planners.rl;

import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.render.Obstacle;
import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes an obstacle object as perceived by the DQN planners.
 * 
 * @author Rafaela Seguro
 *
 */
public class RLObstacle implements Comparable<RLObstacle>{
	
	/** vector that points from the obstacle center to the state*/
	private Vec4 relativeToState = null;
	
	/** vector that points from the obstacle center to the state normalized*/
	private Vec4 normalizedRelativeToState = null;
	
	/** the distance from the obstacle's closest point to the state*/
	private float  distanceToState = 0.0f;
	
	/** the distance from the obstacle's closest point to the state normalized by the environment size*/
	private float  normalizedDistanceToState = 0.0f;
	
	
	/**
	 * Constructs an obstacle instance to be used by a DQN planner. It's characteristics 
	 * are defined relative to a state and are normalized
	 * 
	 * @param the state's position
	 * @param the obstacle
	 * @param the globe
	 * 
	 */
	public RLObstacle(Position position, Obstacle obstacle, PlanningContinuum env) {
		
		Vec4 statePoint = env.transformModelToBoxOrigin(env.getGlobe().computePointFromPosition(position));
		Vec4 obstaclePoint = env.transformModelToBoxOrigin(env.getGlobe().computePointFromPosition(obstacle.getCenter()));
		 
		this.relativeToState = statePoint.subtract3(obstaclePoint);
		this.normalizedRelativeToState = this.relativeToState.normalize3();
		if(obstacle.getExtent(env.getGlobe()) instanceof Cylinder) {
			Cylinder o = (Cylinder) obstacle.getExtent(env.getGlobe());
			this.distanceToState = (float) (this.relativeToState.getLength3() - o.getCylinderRadius());
		} else {
			// TODO: using the box radius isn't really representative of the distance to the obstacle, but I can't think of a better way
			this.distanceToState = (float) (this.relativeToState.getLength3() - obstacle.getExtent(env.getGlobe()).getRadius());
		}
		this.normalizedDistanceToState = (float) (this.distanceToState / env.getDiameter());
	}
	
	
	/**
	 * Gets the normalized vector that points from the obstacle center to the state
	 * 
	 * @return the normalized vector that points from the obstacle center to the state
	 */
	public Vec4 getNormalizedRelativeToState() {
		return this.normalizedRelativeToState;
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
	 * Gets the distance from the obstacle's closest point to the state normalized by the environment size.
	 * 
	 * @return the distance from the obstacle's closest point to the state normalized by the environment size
	 */
	public double getNormalizedDistanceToState() {
		return this.normalizedDistanceToState;
	}
	
	/**
	 * Compares this DQNObstacle to another by their distance to the state
	 * 
	 * @return 0 if the distance is the same; less than 0 if this obstacle is closer to the state; 
	 * greater than 0 if the other obstacle is closer to the state.
	 */
	@Override
	public int compareTo(RLObstacle obstacle) {
		
		return Double.compare(distanceToState, obstacle.getDistanceToState());
	}

}
