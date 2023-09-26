package com.cfar.swim.worldwind.planners.rl;

import java.time.ZonedDateTime;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;

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
public class State {
	
	/** the size of the state ID */
	public static int ID_SIZE = 4;
	
	/** array that represents the state's ID */
	private float[] id = new float[ID_SIZE];
	
//	/** the position relative to the current goal */
//	private PrecisionPosition position;
	
	/** vector that points from state to goal*/
	private Vec4 relativeVector = null;
	
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
	public State(Position position, Position goal, Globe globe) {
		Vec4 statePoint = globe.computePointFromPosition(position);
		Vec4 goalPoint = globe.computePointFromPosition(goal);
		this.relativeVector = goalPoint.subtract3(statePoint);
		this.distanceToGoal = (float) this.relativeVector.getLength3();
		this.createId();
	}
	
	/**
	 * Constructs a state of a reinforcement learning planner from a relative position vector.
	 * 
	 * @param the vector relative to the goal
	 * 
	 */
	public State(Vec4 relativeVector) {
		this.relativeVector = relativeVector;
		this.distanceToGoal = (float) this.relativeVector.getLength3();
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
		id[0] = (float) this.getRelativeVector().x;
		id[1] = (float) this.getRelativeVector().y;
		id[2] = (float) this.getRelativeVector().z;
		id[3] = (float) this.getDistanceToGoal();
	}
	
	/**
	 * Compares this state to another state based on their IDs.
	 * 
	 * @param state the other state
	 * 
	 * @return true if their ID is the same, false otherwise
	 */
	public boolean equals(State state) {

		return this.getId().equals(state.getId());
	}
	
	

}
