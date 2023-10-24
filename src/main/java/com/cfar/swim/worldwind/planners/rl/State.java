package com.cfar.swim.worldwind.planners.rl;

import static org.assertj.core.api.Assertions.assertThatIllegalStateException;

import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.TreeSet;

import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.RLEnvironment;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.CostPolicy;
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
	
	/** the number of considered obstacles for the ID out of the set of close obstacles */
	public static int CONSIDERED_OBSTACLES = 3;
	
	/** the size of the state ID */
	public static int ID_SIZE = 12 + 4*CONSIDERED_OBSTACLES;
	
	/** store's the state's position*/
	private Position position = null;
	
	/** the distance to the environment's axes normalized by their lengths */
	private float[] distancesToEnv = new float[6];
	
	/** normalized vector from goal to state */
	private Vec4 normalizedRelativeGoal = null;
	
	/** the distance from this state to the goal normalized by the maximum distance */
	private float normalizedDistanceToGoal = 0f;
	
	/** the estimated time over of this state */
	private ZonedDateTime eto;
	
	/** array that represents the state's ID */
	private float[] id = new float[ID_SIZE];
	
	/** the set of obstacles from the environment in close proximity to the state*/
	private Set<RLObstacle> obstacles = new TreeSet<RLObstacle>(Comparator.comparingDouble(RLObstacle::getDistanceToState));
	
	/** the applicable cost policy*/
	private CostPolicy costPolicy = CostPolicy.AVERAGE;
	
	/** the current planner's step size normalized */
	double stepSize = 0.0;
	
	/** the vector that represents the current movement direction */
	private Vec4 movVector = null;
	
	

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
	public State(Position position, Position goal, RLEnvironment env, TreeSet<RLObstacle> obstacles, ZonedDateTime eto, Vec4 movVector) {
		
		this.position = position;
		this.eto = eto;
		this.obstacles = obstacles;
		this.costPolicy = env.getCostPolicy();
		this.stepSize = env.toNormalizedDistance(env.getStepSize());

		this.normalizedRelativeGoal = env.getRelativeVector(position, goal).normalize3();
		this.normalizedDistanceToGoal = (float) env.getNormalizedBoxDistance(position, goal);
		this.distancesToEnv = env.getDistancesToEnv(position);
		
		// If it is the first state, the movVector points to the goal
		if(movVector == null) 
			movVector = this.normalizedRelativeGoal.getNegative3();
				
		this.movVector = movVector;
		
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
	 * Gets the normalized vector that points from goal to state
	 * 
	 * @return the normalized vector that points from goal to state
	 */
	public Vec4 getNormalizedRelativeGoal() {
		return this.normalizedRelativeGoal;
	}
	
	/**
	 * Gets the distances from this state to the environment limits (normalized)
	 * 
	 * @return the distances from this state to the environment limits (normalized)
	 */
	public float[] getDistanceToEnv() {
		return this.distancesToEnv;
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
	 * Gets the set of obstacles from the environment in close proximity to the state.
	 * 
	 * @return the set of obstacles from the environment in close proximity to the state
	 */
	public Set<RLObstacle> getObstacles() {
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
	 * Gets the step size for this state.
	 * 
	 * @return the step size
	 */
	public double getStepSize() {
		return this.stepSize;
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
	 * Creates the state's ID based on the stored information
	 */
	public void createId() {
		
		int index = 0;
		
		// Applicable cost policy
		id[index] = (float) costPolicy.getFeatureValue() / 2;
		index++;
		
		// Normalized step size
		id[index] = (float) stepSize;
		index++;
		
		// Distance to environment axes
		for (int i = 0; i < distancesToEnv.length; i++) {
			id[index] =  distancesToEnv[i];
			index++;
		}
		// Information relative to goal
		id[index] = (float) this.getNormalizedRelativeGoal().x;
		id[index+1] = (float) this.getNormalizedRelativeGoal().y;
		id[index+2] = (float) this.getNormalizedRelativeGoal().z;
		id[index+3] = (float) this.getNormalizedDistanceToGoal();
		
		index = index + 4;


		// Adds the obstacles. Puts zero if there are less obstacles than CONSIDERED_OBSTACLES
		Iterator<RLObstacle> itr = obstacles.iterator();
		RLObstacle current = null;
		for (int i=0; i < CONSIDERED_OBSTACLES; i++) {
			if(itr.hasNext()) {
				current =itr.next();
				id[index] = (float) current.getNormalizedRelativeToState().x;
				id[index+1] = (float) current.getNormalizedRelativeToState().y;
				id[index+2] = (float) current.getNormalizedRelativeToState().z;
				id[index+3] = (float) current.getNormalizedDistanceToState();
				index = index + 4;
			} else {
				id[index] = 0.0f;
				id[index+1] = 0.0f;
				id[index+2] = 0.0f;
				id[index+3] = 0.0f;
				index = index + 4;
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

		return Arrays.equals(this.getId(), state.getId());
	}
	
	

}
