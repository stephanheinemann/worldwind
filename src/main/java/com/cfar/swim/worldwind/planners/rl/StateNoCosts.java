package com.cfar.swim.worldwind.planners.rl;

import java.time.ZonedDateTime;


import java.util.Iterator;
import java.util.ArrayList;
import java.util.List;
import java.math.BigDecimal;
import java.math.RoundingMode;

import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
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
	public static int ID_SIZE = 36;
	
	/** array that represents the state's ID */
	private float[][] id = new float[9][4];
	
	/** store's the state's position*/
	private Position position = null;
	
	/** the distance to the environment's corners */
	private List<Vec4> envCorners = new ArrayList<>();
	
	/** vector that points from state to goal*/
	private Vec4 relativeVector = null;
	
	/** vector that points from state to goal*/
	private Vec4 normalizedRelativeVector = null;
	
	/** the distance from this state to the goal (vector length)*/
	private float distanceToGoal = 0f;
	
	/** the estimated time over of this state */
	private ZonedDateTime eto;
	
	/** the cost of this state */
	private double cost;
	
	/**Something to describe the surrounding cost intervals*/
	// TODO: add more environment info
	
	/** indicates if the state has been visited*/
	boolean visited;

	/**
	 * Constructs a state of a reinforcement learning planner from a position.
	 * 
	 * @param position the state's position in globe coordinates
	 * @param goal the goal's position in globe coordinates
	 * @param globe the globe
	 * @param id the state's id
	 * 
	 */
	public StateNoCosts(Position position, Position goal, Vec4[] envCorners, Globe globe) {
		this.position = position;
		Vec4 statePoint = globe.computePointFromPosition(position);
		Vec4 goalPoint = globe.computePointFromPosition(goal);
		this.relativeVector = goalPoint.subtract3(statePoint);
		this.normalizedRelativeVector = this.relativeVector.normalize3();
		this.distanceToGoal = (float) this.relativeVector.getLength3();
		int i = 0;
		for (Vec4 corner : envCorners) {
			this.envCorners.add(statePoint.subtract3(corner));
			i++;
		}
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
	 * Gets the vector that points from state to goal
	 * 
	 * @return the vector that points from state to goal
	 */
	public Vec4 getRelativeToGoal() {
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
	 * Gets the state's ID.
	 * 
	 * @return the state's ID
	 */
	public float[][] getId() {
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
	 * Sets the state to visited or not.
	 * 
	 * @param true if visited, false otherwise
	 */
	public void setVisited(boolean v) {
		this.visited = v;
	}
	
	
	/**
	 * Checks if state has been visited.
	 * 
	 * @return true if state has been visited, false otherwise
	 */
	public boolean isVisited() {
		return this.visited;
	}
	
	/**
	 * Creates the state's ID based on the stored information
	 */
	public void createId() {
		// Relative position and distance to goal rounded up to two decimal places
		//id[0] = BigDecimal.valueOf(this.envCorners[0]).setScale(7, RoundingMode.HALF_EVEN).floatValue();
		int index = 0;
		// Environment corners
		for (int i = 0; i < envCorners.size(); i++) {
			id[index][0] = (float) this.envCorners.get(i).x;
			id[index][1] = (float) this.envCorners.get(i).y;
			id[index][2] = (float) this.envCorners.get(i).z;
			index++;
		}
		// Goal
		id[index][0] = (float) this.getRelativeToGoal().x;
		id[index][1] = (float) this.getRelativeToGoal().y;
		id[index][2] = (float) this.getRelativeToGoal().z;
		id[index][3] = (float) this.getDistanceToGoal();
		
	}
	
	/**
	 * Compares this state to another state based on their IDs.
	 * 
	 * @param state the other state
	 * 
	 * @return true if their ID is the same, false otherwise
	 */
	public boolean equals(StateNoCosts state) {

		return this.getId().equals(state.getId());
	}
	
	

}
