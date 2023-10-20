package com.cfar.swim.worldwind.planners.rl;

import java.time.ZonedDateTime;


import java.util.Iterator;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.math.BigDecimal;
import java.math.RoundingMode;

import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planners.rl.dqn.Helper;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Plane;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;

/**
 * Realizes a state of an RL planner.
 * 
 * @author Rafaela Seguro
 *
 */
public class StateNoCosts {
	
	/** the size of the state ID */
	public static int ID_SIZE = 10;
	
	/** array that represents the state's ID */
	private float[] id = new float[ID_SIZE];
	
	/** store's the state's position*/
	private Position position = null;
	
	/** store's the state's cartesian point */
	private Vec4 statePoint = null;
	
	/** the distance to the environment's axes normalized by their lengths */
	private float[] distanceToEnv = new float[6];
	
	/** vector that points from goal to state */
	private Vec4 relativeGoal = null;
	
	/** vector that points from goal to state */
	private Vec4 normalizedRelativeGoal = null;
	
	/** the distance from this state to the goal (vector length) */
	private float distanceToGoal = 0f;
	
	/** the distance from this state to the goal normalized by the maximum distance */
	private float normalizedDistanceToGoal = 0f;
	
	/** the vector that represents the current movement direction */
	private Vec4 movVector = null;
	

	/**
	 * Constructs a state of a reinforcement learning planner from a position.
	 * 
	 * @param position the state's position in globe coordinates
	 * @param goal the goal's position in globe coordinates
	 * @param globe the globe
	 * @param id the state's id
	 * 
	 */
	public StateNoCosts(Position position, Position goal, PlanningContinuum env, Vec4 movVector) {
		
		this.position = position;
		
		this.statePoint = env.getGlobe().computePointFromPosition(position);
		Vec4 goalPoint = env.getGlobe().computePointFromPosition(goal);
		this.relativeGoal = this.statePoint.subtract3(goalPoint);
		this.normalizedRelativeGoal = this.relativeGoal.normalize3();
		this.distanceToGoal = (float) this.relativeGoal.getLength3();
		this.normalizedDistanceToGoal = (float) (this.distanceToGoal / env.getDiameter());
		
		// If it is the first state, the movVector points to the goal
		if(movVector == null) 
			movVector = this.normalizedRelativeGoal.getNegative3();
				
		this.movVector = movVector;
		
		// Gets distance to environment axes
		this.distanceToEnv[0] = (float) (this.statePoint.x / env.getDiameter());
		this.distanceToEnv[1] = (float) ((env.getRLength() - this.statePoint.x) / env.getDiameter());
		this.distanceToEnv[2] = (float) (this.statePoint.y / env.getDiameter());
		this.distanceToEnv[3] = (float) ((env.getSLength() - this.statePoint.y) / env.getDiameter());
		this.distanceToEnv[4] = (float) (this.statePoint.z / env.getDiameter());
		this.distanceToEnv[5] = (float) ((env.getTLength() - this.statePoint.z) / env.getDiameter());
		this.createId();
	}
	
	
	/**
	 * Gets the state's position
	 * 
	 * @return the state's position
	 */
	public Position getPosition() {
		return this.position;
	}
	
	/**
	 * Gets the state's point in cartesian coordinates
	 * 
	 * @return the state's point in cartesian coordinates
	 */
	public Vec4 getStatePoint() {
		return this.statePoint;
	}
	
	/**
	 * Gets the vector that points from goal to state
	 * 
	 * @return the vector that points from goal to state
	 */
	public Vec4 getRelativeGoal() {
		return this.relativeGoal;
	}
	
	/**
	 * Gets the vector that points from state to goal normalized
	 * 
	 * @return the vector that points from state to goal normalized
	 */
	public Vec4 getNormalizedRelativeGoal() {
		return this.normalizedRelativeGoal;
	}
	
	/**
	 * Gets the distance from this state to the goal (vector length)
	 * 
	 * @return the distance from this state to the goal (vector length)
	 */
	public double getDistanceToGoal() {
		return this.distanceToGoal;
	}
	
	/**
	 * Gets the distance from this state to the goal normalized by the maximum distance
	 * 
	 * @return the distance from this state to the goal normalized by the maximum distance
	 */
	public double getNormalizedDistanceToGoal() {
		return this.normalizedDistanceToGoal;
	}
	
	/**
	 * Gets the vector that represents the current movement direction
	 * 
	 * @return the vector that represents the current movement direction
	 */
	public Vec4 getMovVector() {
		return this.movVector;
	}
	
	/**
	 * Gets the state's ID.
	 * 
	 * @return the state's ID
	 */
	public float[] getId() {
		return this.id;
	}
	

	
	/**
	 * Creates the state's ID based on the stored information
	 */
	public void createId() {
		// Relative position and distance to goal rounded up to two decimal places
		//id[0] = BigDecimal.valueOf(this.envCorners[0]).setScale(7, RoundingMode.HALF_EVEN).floatValue();
		int index = 0;
		
		// Distance to environment axes
		for (int i = 0; i < distanceToEnv.length; i++) {
			id[index] =  distanceToEnv[i];
			index++;
		}
		// Information relative to goal
		id[index] = (float) this.getNormalizedRelativeGoal().x;
		id[index+1] = (float) this.getNormalizedRelativeGoal().y;
		id[index+2] = (float) this.getNormalizedRelativeGoal().z;
		id[index+3] = (float) this.getNormalizedDistanceToGoal();
		
	}
	
	/**
	 * Compares this state to another state based on their IDs.
	 * 
	 * @param state the other state
	 * 
	 * @return true if their ID is the same, false otherwise
	 */
	public boolean equals(StateNoCosts state) {

		return Arrays.equals(this.getId(), state.getId());
	}
	
	

}
