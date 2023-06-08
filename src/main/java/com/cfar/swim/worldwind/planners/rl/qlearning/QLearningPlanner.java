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
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;
import com.cfar.swim.worldwind.planners.rl.QLine;
import com.cfar.swim.worldwind.planners.rl.RLWaypoint;
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
	protected Map<RLWaypoint, QLine> qTable = new TreeMap<>();
	
	/** the start waypoint */
	private RLWaypoint start = null;
	
	/** the goal waypoint */     
	private RLWaypoint goal = null;
	
	/** the current waypoint (state) */     
	private RLWaypoint state = null;
	
	/** the transition waypoint (next state) */     
	private RLWaypoint nextState = null;
	
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
	 * Creates a waypoint at a specified position and adds it 
	 * as a state to the Q-table. Also creates set of neighbor waypoints.
	 * 
	 * @param position the position
	 * 
	 * @return the waypoint at the specified position
	 */
	protected RLWaypoint createWaypoint(Position position) {
		RLWaypoint wp = new RLWaypoint(position);
		
		// only adds new waypoint to the Q-table if it is not there yet
		if (!(qTable.containsKey(wp))) {
			Set<Position> neighbors = this.getEnvironment().getNeighbors(wp);
			// if a position has no neighbors, then it is not a waypoint in the
			// environment and its adjacent waypoints have to be determined for
			// initial expansion
			if (neighbors.isEmpty()) {
				neighbors = this.getEnvironment().getAdjacentWaypointPositions(wp);
			}
			wp.setNeighbors(neighbors);
			QLine newLine =  new QLine(wp, neighbors.size());
			newLine.initQValues();
			qTable.put(wp, newLine);
		}
		
		return wp;
	}
	
	/**
	 * Gets the start waypoint of this  planner.
	 * 
	 * @return the start waypoint of this planner
	 */
	protected RLWaypoint getStart() {
		return this.start;
	}
	
	/**
	 * Sets the start waypoint of this  planner.
	 * 
	 * @param start the start waypoint of this planner
	 */
	protected void setStart(RLWaypoint start) {
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
	protected RLWaypoint getGoal() {
		return this.goal;
	}
	
	/**
	 * Sets the goal waypoint of this planner.
	 * 
	 * @param goal the goal waypoint of this planner
	 */
	protected void setGoal(RLWaypoint goal) {
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
	 * Gets the current state.
	 * 
	 * @return the state 
	 */
	protected RLWaypoint getState() {
		return this.state;
	}
	
	/**
	 * Sets the current state.
	 * 
	 * @param state the current state
	 */
	protected void setState(RLWaypoint state) {
		this.state = state;
	}
	
	/**
	 * Gets the next state.
	 * 
	 * @return the next state
	 */
	protected RLWaypoint getNextState() {
		return this.nextState;
	}
	
	/**
	 * Sets the next state of this planner.
	 * 
	 * @param nextState the next state of this planner
	 */
	protected void setNextState(RLWaypoint nextState) {
		this.nextState = nextState;
	}
	
	/**
	 * Gets the chosen action index.
	 * 
	 * @return the chosen action index
	 */
	protected int getAction() {
		return this.action;
	}
	
	/**
	 * Sets the chosen action index.
	 * 
	 * @param action the chosen action index
	 */
	protected void setAction(int action) {
		this.action = action;
	}
	
	/**
	 * Gets the reward.
	 * 
	 * @return the reward
	 */
	protected int getReward() {
		return this.reward;
	}
	
	/**
	 * Sets the reward.
	 * 
	 * @param reward the reward
	 */
	protected void setReward(int reward) {
		this.reward = reward;
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
	
	/**
	 * Initializes this Q-Learning planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.setStart(null);
		this.setGoal(null);
		
		this.setStart(this.createWaypoint(origin));
		this.getStart().setEto(etd);
		this.getStart().setPoi(true);
		
		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setPoi(true);
		
		// the adjacent waypoints to the origin
		this.setStartRegion(this.getEnvironment().getAdjacentWaypointPositions(origin)
				.stream()
				.map(PrecisionPosition::new)
				.collect(Collectors.toSet()));
		
		// the adjacent waypoints to the destination
		this.setGoalRegion(this.getEnvironment().getAdjacentWaypointPositions(destination)
				.stream()
				.map(PrecisionPosition::new)
				.collect(Collectors.toSet()));
	}
	
	/**
	 * Calculates the reward based on the next state (only takes into account 
	 * distance to goal for now)
	 */
	public void calculateReward() {
		// If it has reached the goal region, the reward is 100
		if (isInGoalRegion(this.nextState.getPrecisionPosition())) {
			setReward(100);
			return;
		}
		
		// Calculate ETA for state and next_state
		Path leg1 = new Path(this.state, this.goal);
		Path leg2 = new Path(this.nextState, this.goal);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		ZonedDateTime end1 = capabilities.getEstimatedTime(leg1, globe, this.state.getEto());
		ZonedDateTime end2 = capabilities.getEstimatedTime(leg2, globe, this.nextState.getEto());
		
		// If ETA of next_state is higher, reward is -10, else it is 0
		if (end2.isAfter(end1)) {
			setReward(-100);
		} else {
			setReward(0);
		}
	}
	
	/**
	 * Calculates the new Q value based on the Q-Learning equation:
	 * newQ(s,a) = Q(s,a) + alpha.( R(s,a) + gamma.max(Q(s',a') - Q(s,a))
	 * and updates the table
	 */
	public void updateQ() {
		// Calculates new Q value for state and action pair
		double newQ = qTable.get(this.state).getQValue(this.action) + this.alpha 
				* (this.reward + this.gamma * qTable.get(this.nextState).getMaxQValue() 
						- qTable.get(this.state).getQValue(this.action));
		// Updates the corresponding Q-table entry
		qTable.get(this.state).updateQValue(this.action, newQ);
	}
	
	/**
	 * Runs the training to learn the Q-values
	 */
	public void train() {
		// Loop to run all the training episodes
		int ep = 0;
		while (ep < this.maxEpisodes) {
			setState(this.getStart());
			// Loop to go from start to goal
			for (int j = 0; j < this.maxSteps; j++) {
				// Chooses action according to Q-table
				setAction(this.qTable.get(this.state).getMaxQValue());
				// Updates next state
				RLWaypoint ns = createWaypoint(this.state.getNeighbor(getAction()));
				setNextState(ns);
				// Gets reward
				calculateReward();
				// Updates Q-Table
				updateQ();
				// Checks if it reached goal and, if yes, finish loop and if yes increments ep
				if (isInGoalRegion(this.nextState.getPrecisionPosition())) {
					ep++;
					break;
				}
			}
		}
		
	}
	
	/**
	 * Computes a plan according to the trained Q-Table.
	 */
	protected void compute() {
		this.clearWaypoints();
		this.getWaypoints().addLast(this.start);
		setState(this.start);
		// TODO: maybe have max steps here as well, but have to see what to do
		// if plan is empty then
		while (true) {
			// Chooses action according to Q-table
			setAction(this.qTable.get(this.state).getMaxQValue());
			// Updates next state
			RLWaypoint ns = createWaypoint(this.state.getNeighbor(getAction()));
			this.getWaypoints().addLast(ns);
			setNextState(ns);
			// Gets reward
			calculateReward();
			// Updates Q-Table
			updateQ();
			// Checks if it reached goal and, if yes, finish loop
			if (isInGoalRegion(this.nextState.getPrecisionPosition())) {
				break;
			}
		}
	}
	
	/**
	 * Plans a part of a trajectory.
	 * 
	 * @param partIndex the index of the part
	 * 
	 * @return the planned part of a trajectory
	 * 
	 */
	protected Trajectory planPart(int partIndex) {
		this.train();
		this.compute();
		return this.createTrajectory();
	}
	
	

	/**
	 * Plans a trajectory from an origin to a destination at a specified
	 * estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with
	 *         the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);
		Trajectory trajectory = this.planPart(0);
		this.revisePlan(trajectory);
		return trajectory;
	}
	
	/**
	 * Plans a trajectory from an origin to a destination along waypoints at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination along
	 *         the waypoints with the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		RLWaypoint currentOrigin = new RLWaypoint(origin);
		ZonedDateTime currentEtd = etd;
		
		// collect intermediate destinations
		ArrayList<RLWaypoint> destinations = waypoints.stream()
				.map(RLWaypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new RLWaypoint(destination));
		
		// plan and concatenate partial trajectories
		for (int partIndex = 0; partIndex < destinations.size(); partIndex++) {
			RLWaypoint currentDestination = destinations.get(partIndex);
			if (!(currentOrigin.equals(currentDestination))) {
				
				/* 
				 * Each part of a multi-part plan has to be computed completely
				 * in order to finalize the ETO of the goal waypoint which
				 * becomes the start waypoint of the next part and the basis
				 * for any subsequent plan revisions. A possible repair of one
				 * part requires the re-computation of all subsequent parts.
				 * This cost-greedy solution does not necessarily result in
				 * optimality with respect to the overall cost of the computed
				 * multi-part plan.
				 * 
				 * https://github.com/stephanheinemann/worldwind/issues/24
				 */
				
				// plan partial trajectory
				this.initialize(currentOrigin, currentDestination, currentEtd);
				
				this.planPart(partIndex);
				
				if ((!this.hasWaypoints())
						|| (!this.getWaypoints().getLast().equals(currentDestination))) {
					// if no plan could be found, return an empty trajectory
					Trajectory empty = new Trajectory();
					this.revisePlan(empty);
					return empty;
				} else {
					// revise growing trajectory for each part
					this.revisePlan(this.createTrajectory());
					currentOrigin = this.getGoal();
					currentEtd = currentOrigin.getEto();
				}
			}
		}
		
		return this.createTrajectory();
	}
	
	/**
	 * Indicates whether or not this forward A* planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a planning grid or roadmap,
	 *         false otherwise
	 *         
	 * @see PlanningGrid
	 * @see PlanningRoadmap
	 */
	@Override
	public boolean supports(Environment environment) {
		boolean supports = super.supports(environment);
		
		if (supports) {
			supports = (environment instanceof PlanningGrid) ||
					(environment instanceof PlanningRoadmap);
		}
		
		return supports;
	}
	
}

