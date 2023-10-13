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
	
	/** store's the state's point in box coordinates*/
	private Vec4 boxStatePoint = null;
	
	/** the distance to the environment's axes normalized by their lengths */
	private float[] distanceToEnv = new float[6];
	
	/** vector that points from state to goal*/
	private Vec4 relativeVector = null;
	
	/** vector that points from state to goal*/
	private Vec4 normalizedRelativeVector = null;
	
	/** the distance from this state to the goal (vector length) */
	private float distanceToGoal = 0f;
	
	/** the distance from this state to the goal normalized by the maximum distance */
	private float normalizedDistanceToGoal = 0f;
	
	/** the estimated time over of this state */
	private ZonedDateTime eto;
	
	/** the cost of this state */
	private double cost;
	
	/**Something to describe the surrounding cost intervals*/
	// TODO: add more environment info
	

	/**
	 * Constructs a state of a reinforcement learning planner from a position.
	 * 
	 * @param position the state's position in globe coordinates
	 * @param goal the goal's position in globe coordinates
	 * @param globe the globe
	 * @param id the state's id
	 * 
	 */
	public StateNoCosts(Position position, Position goal, PlanningContinuum env) {
		this.position = position;
		Vec4 statePoint = env.getGlobe().computePointFromPosition(position);
		boxStatePoint = env.transformModelToBoxOrigin(statePoint);
		Vec4 goalPoint = env.getGlobe().computePointFromPosition(goal);
		Vec4 boxGoalPoint = env.transformModelToBoxOrigin(goalPoint);
		this.relativeVector = boxGoalPoint.subtract3(boxStatePoint);
		this.normalizedRelativeVector = this.relativeVector.normalize3();
		this.distanceToGoal = (float) this.relativeVector.getLength3();
		this.normalizedDistanceToGoal = (float) (this.distanceToGoal / env.getDiameter());
		
		// Gets distance to environment axes
		this.distanceToEnv[0] = (float) (boxStatePoint.x / env.getRLength());
		this.distanceToEnv[1] = (float) ((env.getRLength() - boxStatePoint.x) / env.getRLength());
		this.distanceToEnv[2] = (float) (boxStatePoint.y / env.getSLength());
		this.distanceToEnv[3] = (float) ((env.getSLength() - boxStatePoint.y) / env.getSLength());
		this.distanceToEnv[4] = (float) (boxStatePoint.z / env.getTLength());
		this.distanceToEnv[5] = (float) ((env.getTLength() - boxStatePoint.z) / env.getTLength());
		this.createId();
	}
	
//	/**
//	 * Constructs a state of a reinforcement learning planner from a relative position vector.
//	 * 
//	 * @param the vector relative to the goal
//	 * 
//	 */
//	public StateNoCosts(Vec4 relativeVector) {
//		this.relativeVector = relativeVector;
//		this.normalizedRelativeVector = this.relativeVector.normalize3();
//		this.distanceToGoal = (float) this.relativeVector.getLength3();
//		this.createId();
//	}
	
	/**
	 * Gets the state's position
	 * 
	 * @return the state's position
	 */
	public Position getPosition() {
		return this.position;
	}
	
	/**
	 * Gets the state's position
	 * 
	 * @return the state's position
	 */
	public Vec4 getBoxStatePoint() {
		return this.boxStatePoint;
	}
	
	/**
	 * Gets the vector that points from state to goal
	 * 
	 * @return the vector that points from state to goal
	 */
	public Vec4 getRelativeVector() {
		return this.relativeVector;
	}
	
	/**
	 * Gets the vector that points from state to goal normalized
	 * 
	 * @return the vector that points from state to goal normalized
	 */
	public Vec4 getNormalizedRelativeVector() {
		return this.normalizedRelativeVector;
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
	 * Gets the state's ID.
	 * 
	 * @return the state's ID
	 */
	public float[] getId() {
		return this.id;
	}
	
	/**
	 * Sets the state's ETO.
	 * 
	 * @param eto state's ETO
	 */
	public void setEto(ZonedDateTime eto) {
		this.eto = eto;
	}
	
	
	/**
	 * Gets the state's ETO.
	 * 
	 * @return the state's ETO
	 */
	public ZonedDateTime getEto() {
		return this.eto;
	}
	
	/**
	 * Sets the state's cost.
	 * 
	 * @param cost state's cost
	 */
	public void setCost(double cost) {
		this.cost = cost;
	}
	
	
	/**
	 * Gets the state's cost.
	 * 
	 * @return the state's cost
	 */
	public double getCost() {
		return this.cost;
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
		id[index] = (float) this.getNormalizedRelativeVector().x;
		id[index+1] = (float) this.getNormalizedRelativeVector().y;
		id[index+2] = (float) this.getNormalizedRelativeVector().z;
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
