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
	public static int ID_SIZE = 12 + 4*CONSIDERED_OBSTACLES;
	
	/** store's the state's position*/
	private PrecisionPosition position = null;
	
	/** store's the state's point in box coordinates*/
	private Vec4 statePoint = null;
	
	/** store's the state's point in box coordinates*/
	private Vec4 boxStatePoint = null;
	
	/** the distance to the environment's axes normalized by their lengths */
	private float[] distanceToEnv = new float[6];
	
	/** represents the goal position relative to the state */
	private Vec4 relativeGoal = null;
	
	/** normalized vector from goal to state */
	private Vec4 normalizedRelativeGoal = null;
	
	/** the distance from this state to the goal (vector length) */
	private float distanceToGoal = 0f;
	
	/** the distance from this state to the goal normalized by the maximum distance */
	private float normalizedDistanceToGoal = 0f;
	
	/** the estimated time over of this state */
	private ZonedDateTime eto;
	
	/** array that represents the state's ID */
	private float[] id = new float[ID_SIZE];
	
	/** the set of obstacles from the environment in close proximity to the state*/
	private Set<DQNObstacle> obstacles = new TreeSet<DQNObstacle>(Comparator.comparingDouble(DQNObstacle::getDistanceToState));
	
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
	public State(PrecisionPosition position, PrecisionPosition goal, PlanningContinuum env, TreeSet<DQNObstacle> obstacles, CostPolicy costPolicy, 
			ZonedDateTime eto, double stepSize, Vec4 movVector) {
		this.position = position;
		this.eto = eto;
		this.obstacles = obstacles;
		this.costPolicy = costPolicy;
		this.stepSize = stepSize / env.getDiameter();
		
		this.position = position;
		this.statePoint = env.getGlobe().computePointFromPosition(position);
		this.boxStatePoint = env.transformModelToBoxOrigin(this.statePoint);
		Vec4 goalPoint = env.getGlobe().computePointFromPosition(goal);
		Vec4 boxGoalPoint = env.transformModelToBoxOrigin(goalPoint);
		this.relativeGoal = this.boxStatePoint.subtract3(boxGoalPoint);
		this.normalizedRelativeGoal = this.relativeGoal.normalize3();
		this.distanceToGoal = (float) this.relativeGoal.getLength3();
		this.normalizedDistanceToGoal = (float) (this.distanceToGoal / env.getDiameter());
		
		// If it is the first state, the movVector points to the goal
		if(movVector == null) 
			movVector = this.normalizedRelativeGoal.getNegative3();
				
		this.movVector = movVector;
				
		// Gets distance to environment axes
		this.distanceToEnv[0] = (float) (this.boxStatePoint.x / env.getDiameter());
		this.distanceToEnv[1] = (float) ((env.getRLength() - this.boxStatePoint.x) / env.getDiameter());
		this.distanceToEnv[2] = (float) (this.boxStatePoint.y / env.getDiameter());
		this.distanceToEnv[3] = (float) ((env.getSLength() - this.boxStatePoint.y) / env.getDiameter());
		this.distanceToEnv[4] = (float) (this.boxStatePoint.z / env.getDiameter());
		this.distanceToEnv[5] = (float) ((env.getTLength() - this.boxStatePoint.z) / env.getDiameter());
		
		this.createId();
	}
	
	/**
	 * Gets the state's position
	 * 
	 * @return the state's position
	 */
	public PrecisionPosition getPosition() {
		return this.position;
	}
	
	/**
	 * Gets the state's point in box coordinates
	 * 
	 * @return the state's point in box coordinates
	 */
	public Vec4 getStatePoint() {
		return this.boxStatePoint;
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
	 * Gets the vector that points from goal to state
	 * 
	 * @return the vector that points from goal to state
	 */
	public Vec4 getrelativeGoal() {
		return this.relativeGoal;
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
	
	
//	/**
//	 * Calculates the minimum distance from axes, obstacles or goal to the state.
//	 * 
//	 * @return the minimum distance (normalized)
//	 */
//	public float getMinDistance() {
//		float[] distances = new float[7 + obstacles.size()];
//		for (int i=0; i<6; i++)
//			distances[i] = this.distanceToEnv[i];
//		distances[6] = this.normalizedDistanceToGoal;
//		Iterator<DQNObstacle> itr = obstacles.iterator();
//		DQNObstacle current = null;
//		int i = 1;
//		while(itr.hasNext()) {
//			current =itr.next();
//			distances[6+i] = (float) current.getNormalizedDistanceToState();
//			i++;
//		}
//		
//		float minDistance = distances[0];
//		for (int j=0; j<distances.length; j++) {
//			if (distances[j] < minDistance)
//				minDistance = distances[j];
//		}
//		
//		return minDistance;
//	}
	
	
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
		for (int i = 0; i < distanceToEnv.length; i++) {
			id[index] =  distanceToEnv[i];
			index++;
		}
		// Information relative to goal
		id[index] = (float) this.getNormalizedRelativeGoal().x;
		id[index+1] = (float) this.getNormalizedRelativeGoal().y;
		id[index+2] = (float) this.getNormalizedRelativeGoal().z;
		id[index+3] = (float) this.getNormalizedDistanceToGoal();
		
		index = index + 4;


		// Adds the obstacles. Puts zero if there are less obstacles than CONSIDERED_OBSTACLES
		Iterator<DQNObstacle> itr = obstacles.iterator();
		DQNObstacle current = null;
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
