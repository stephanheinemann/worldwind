package com.cfar.swim.worldwind.planners.rl.qlearning;

import java.util.ArrayList;
import java.time.ZonedDateTime;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.environments.PlanningRoadmap;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.rl.QLine;
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
* Realizes a basic reinforcement learning planner, using Q-Learning that plans a trajectory of an aircraft
* in an environment considering a local cost and risk policy.
* 
* @author Rafaela Seguro
*
*/

public class QLearningPlanner extends AbstractPlanner {
	
	/** the Q-table map, where each element represents a line corresponding to a state; the key is the state */
	protected Map<Waypoint, QLine> qTable = new TreeMap<>();
	
	/** the start waypoint */
	private Waypoint start = null;
	
	/** the goal waypoint */     
	private Waypoint goal = null;
	
	/** the current waypoint (state) */     
	private Waypoint state = null;
	
	/** the transition waypoint (next state) */     
	private Waypoint nextState = null;
	
	/** the index corresponding to the chosen action */
	private int action = 0;
	
	/** the reward corresponding to the chosen action */
	private int reward = 0;
	
	/** the start region */
	private Set<PrecisionPosition> startRegion;
	
	/** the goal region */
	private Set<PrecisionPosition> goalRegion;
	
	/** the maximum number of episodes in the learning phase of this planner */
	private int maxEpisodes = 3_000; 
	
	/** the maximum number of steps from start to goal of this planner */
	private int maxSteps = 50; 
	
	/** the maximum number of steps from start to goal of this planner */
	private double alpha = 0.6; 
	
	/** the maximum number of steps from start to goal of this planner */
	private double gamma = 0.9; 
	
	/**
	 * Constructs a Q-Learning planner for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public QLearningPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		// TODO Auto-generated constructor stub
	}
	
	@Override
	public String getId() {
		// TODO Auto-generated method stub
		return Specification.PLANNER_QLP_ID;
	}
	
	/**
	 * Gets the start waypoint of this  planner.
	 * 
	 * @return the start waypoint of this planner
	 */
	protected Waypoint getStart() {
		return this.start;
	}
	
	/**
	 * Sets the start waypoint of this  planner.
	 * 
	 * @param start the start waypoint of this planner
	 */
	protected void setStart(Waypoint start) {
		this.start = start;
	}
	
	/**
	 * Determines whether or not this planner has a
	 * start waypoint.
	 * 
	 * @return true if this planner has a start waypoint,
	 *         false otherwise
	 */
	protected boolean hasStart() {
		return (null != this.start);
	}
	
	/**
	 * Gets the goal waypoint of this planner.
	 * 
	 * @return the goal waypoint of this planner
	 */
	protected Waypoint getGoal() {
		return this.goal;
	}
	
	/**
	 * Sets the goal waypoint of this planner.
	 * 
	 * @param goal the goal waypoint of this planner
	 */
	protected void setGoal(Waypoint goal) {
		this.goal = goal;
	}
	
	/**
	 * Determines whether or not this planner has a
	 * goal waypoint.
	 * 
	 * @return true if this planner has a goal waypoint,
	 *         false otherwise
	 */
	protected boolean hasGoal() {
		return (null != this.goal);
	}
	
	/**
	 * Gets the start region of this planner.
	 * 
	 * @return the start region of this planner
	 */
	protected Set<PrecisionPosition> getStartRegion() {
		return this.startRegion;
	}
	
	/**
	 * Sets the start region of this planner.
	 * 
	 * @param startRegion the start region of this planner
	 */
	protected void setStartRegion(Set<PrecisionPosition> startRegion) {
		this.startRegion = startRegion;
	}
	
	/**
	 * Determines whether or not a position is within the start region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the start region, false otherwise
	 */
	protected boolean isInStartRegion(Position position) {
		return this.startRegion.contains(position);
	}
	
	/**
	 * Gets the goal region of this planner.
	 * 
	 * @return the goal region of this planner
	 */
	protected Set<PrecisionPosition> getGoalRegion() {
		return this.goalRegion;
	}
	
	/**
	 * Sets the goal region of this planner.
	 * 
	 * @param goalRegion the goal region of this planner
	 */
	protected void setGoalRegion(Set<PrecisionPosition> goalRegion) {
		this.goalRegion = goalRegion;
	}
	
	/**
	 * Determines whether or not a position is within the goal region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the goal region, false otherwise
	 */
	protected boolean isInGoalRegion(Position position) {
		return this.goalRegion.contains(position);
	}

	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		AbstractPlanner planner = new ForwardAStarPlanner(getAircraft(), getEnvironment());
		return planner.plan(origin, destination, etd);
	}



	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		// TODO Auto-generated method stub
		return null;
	}
	
	

}
