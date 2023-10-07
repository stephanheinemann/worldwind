package com.cfar.swim.worldwind.planners.rl;

import java.time.ZonedDateTime;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.TreeSet;

import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.planners.rl.dqn.DQNObstacle;

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
	
	/** the number of considered obstacles for the ID out of the set of close obstacles */
	public static int CONSIDERED_OBSTACLES = 3;
	
	/** the size of the state ID */
	public static int ID_SIZE = 5 + 4*CONSIDERED_OBSTACLES;
	
	/** array that represents the state's ID */
	private float[] id = new float[ID_SIZE];
	
	/** store's the state's position*/
	private Position position = null;
	
	/** store's the state's ETO*/
	private ZonedDateTime eto = null;
	
	/** vector that points from state to goal*/
	private Vec4 relativeToGoal = null;
	
	/** the distance from this state to the goal (vector length)*/
	private float distanceToGoal = 0f;
	
	/** the set of obstacles from the environment in close proximity to the state*/
	private Set<DQNObstacle> obstacles = new TreeSet<DQNObstacle>(Comparator.comparingDouble(DQNObstacle::getDistanceToState));
	
	/** the applicable cost policy*/
	private CostPolicy costPolicy = CostPolicy.AVERAGE;
	
	/** indicates if the state has been visited*/
	boolean visited;
	

	/**
	 * Constructs a state of a reinforcement learning planner from a position.
	 * 
	 * @param position the state's position in globe coordinates
	 * @param goal the goal's position in globe coordinates
	 * @param obstacles the set of obstacles from the environment in close proximity to the state
	 * @param costPolicy the DQN planner's cost policy
	 * @param globe the globe
	 * 
	 */
	public State(Position position, Position goal, Set<DQNObstacle> obstacles, CostPolicy costPolicy, ZonedDateTime eto, Globe globe) {
		this.position = position;
		this.eto = eto;
		Vec4 statePoint = globe.computePointFromPosition(position);
		Vec4 goalPoint = globe.computePointFromPosition(goal);
		this.relativeToGoal = goalPoint.subtract3(statePoint);
		this.distanceToGoal = (float) this.relativeToGoal.getLength3();
		this.obstacles = obstacles;
		this.costPolicy = costPolicy;
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
	 * Gets the state's ETO
	 * 
	 * @return the state's ETO
	 */
	public ZonedDateTime getEto() {
		return this.eto;
	}
	
	
	/**
	 * Gets the vector that points from state to goal
	 * 
	 * @return the vector that points from state to goal
	 */
	public Vec4 getrelativeToGoal() {
		return this.relativeToGoal;
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
	 * Gets the set of obstacles from the environment in close proximity to the state.
	 * 
	 * @return the set of obstacles from the environment in close proximity to the state
	 */
	public Set<DQNObstacle> getObstacles() {
		return this.obstacles;
	}

	
	/**
	 * Gets the applicable cost policy.
	 * 
	 * @return the applicable cost policy
	 */
	public CostPolicy getCostPolicy() {
		return this.costPolicy;
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
		
		// Applicable cost policy
		id[0] = (float) costPolicy.getFeatureValue();
		
		// Relative position and distance to goal
		id[1] = (float) this.getrelativeToGoal().x;
		id[2] = (float) this.getrelativeToGoal().y;
		id[3] = (float) this.getrelativeToGoal().z;
		id[4] = (float) this.getDistanceToGoal();

		// Adds the obstacles. Puts zero if there are less obstacles than CONSIDERED_OBSTACLES
		Iterator<DQNObstacle> itr = obstacles.iterator();
		DQNObstacle current = null;
		for (int i=0; i < CONSIDERED_OBSTACLES; i++) {
			if(itr.hasNext()) {
				current =itr.next();
				id[5+(4*i)] = (float) current.getrelativeToState().x;
				id[6+(4*i)] = (float) current.getrelativeToState().y;
				id[7+(4*i)] = (float) current.getrelativeToState().z;
				id[8+(4*i)] = (float) current.getDistanceToState();
			} else {
				id[5+(4*i)] = 0.0f;
				id[6+(4*i)] = 0.0f;
				id[7+(4*i)] = 0.0f;
				id[8+(4*i)] = 0.0f;
			}
		}
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
