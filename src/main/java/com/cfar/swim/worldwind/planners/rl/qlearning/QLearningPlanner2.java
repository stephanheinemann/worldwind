package com.cfar.swim.worldwind.planners.rl.qlearning;

import java.util.ArrayList;


import java.awt.Color;
import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;
import java.time.temporal.TemporalUnit;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
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
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.environments.PlanningRoadmap;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;
import com.cfar.swim.worldwind.planners.rl.Plot;
import com.cfar.swim.worldwind.planners.rl.QLine;
import com.cfar.swim.worldwind.planners.rl.RLWaypoint;
import com.cfar.swim.worldwind.planners.rl.State;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
//import com.cfar.swim.worldwind.tests.Plot;
import com.cfar.swim.worldwind.util.Identifiable;

import ch.qos.logback.core.joran.action.Action;
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

public class QLearningPlanner2 extends AbstractPlanner {
	
	/** the Q-table map, where each element represents a line corresponding to a state; the key is the state */
	protected Map<Integer, QLine> qTable = new TreeMap<>();
	
	/** the set of already discovered waypoints */
	protected Set<RLWaypoint> discovered = new HashSet<>();
	
	/** the start waypoint */
	private RLWaypoint start = null;
	
	/** the goal waypoint */     
	private RLWaypoint goal = null;
	
	/** the intermediate plan waypoints */     
	private List<RLWaypoint> interWaypoints = new ArrayList<>();
	
	/** the current state */     
	private State state = null;
	
	/** the current waypoint (position)*/     
	private RLWaypoint currentWp = null;
	
	/** the next state */     
	private State nextState = null;
	
	/** the next waypoint (position) */     
	private RLWaypoint nextWp = null;
	
	/** stores the plan's ETD */
	private ZonedDateTime etd;
	
	/** the chosen action */
	private Action action = null;
	
	/** the reward corresponding to the chosen action */
	private double reward = 0;
	
	/** the start region */
	private Set<PrecisionPosition> startRegion;
	
	/** the goal region */
	private Set<PrecisionPosition> goalRegion;
	
	/** the maximum number of episodes in the learning phase of this planner */
	private int maxEpisodes = 300; 
	
	/** the maximum number of steps from start to goal of this planner */
	private int maxSteps = 50; 
	
	/** the maximum number of steps from start to goal of this planner */
	private double alpha = 0.8; 
	
	/** the minimum, maximum and current epsilon value to implement decaying e-greedy */
	private double minEpsilon = 0.0; 
	private double maxEpsilon = 1; 
	private double epsilon = 1; 
	
	/** the maximum number of steps from start to goal of this planner */
	private double gamma = 0.9; 
	
	/** waypoint counter */
	private int wpId = 0;
	
	/** state counter */
	private int stateId = 0;

	
	/**
	 * Constructs a Q-Learning planner for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public QLearningPlanner2(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	@Override
	public String getId() {
		// TODO Auto-generated method stub
		return Specification.PLANNER_QLP_ID;
	}
	
	/**
	 * Clears the discovered waypoints set
	 */
	protected void clearDiscovered() {
		this.discovered.clear();
		wpId = 0;
	}
	
	/**
	 * Adds a discovered waypoint of this planner to the set.
	 * 
	 * @param the discovered waypoint
	 */
	protected void addDiscovered(RLWaypoint wp) {
		this.discovered.add(wp);
	}
	
	/**
	 * Finds an RL waypoint in the discovered set of this QLearning planner, if
	 * present.
	 * 
	 * @param waypoint the RL waypoint to be found
	 * 
	 * @return the found RL waypoint, if present 
	 */
	protected Optional<? extends RLWaypoint> findDiscovered(RLWaypoint waypoint) {
		return this.discovered.stream()
				.filter(visitedWaypoint -> visitedWaypoint.equals(waypoint))
				.findFirst();
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
	 * Clears the intermediate waypoints list of this planner.
	 */
	protected void clearInterWaypoints() {
		this.interWaypoints.clear();
	}
	
	/**
	 * Gets the intermediate waypoints of this planner.
	 * 
	 * @return the intermediate waypoints of this planner
	 */
	protected List<RLWaypoint> getInterWaypoints() {
		return this.interWaypoints;
	}
	
	/**
	 * Adds an intermediate waypoint of this planner to the list.
	 * 
	 * @param the intermediate waypoint
	 */
	protected void addInterWaypoint(RLWaypoint wp) {
		this.interWaypoints.add(wp);
	}
	
	/**
	 * Determines whether or not a position is within a waypoint's region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the goal region, false otherwise
	 */
	protected boolean isInWaypointRegion(Position position, RLWaypoint waypoint) {
		return this.getEnvironment().getAdjacentWaypointPositions(waypoint)
				.stream()
				.map(PrecisionPosition::new)
				.collect(Collectors.toSet()).contains(position);
	}
	
	/**
	 * Gets the current state.
	 * 
	 * @return the state 
	 */
	protected State getState() {
		return this.state;
	}
	
	/**
	 * Sets the current state.
	 * 
	 * @param state the current state
	 */
	protected void setState(State state) {
		this.state = state;
	}
	
	/**
	 * Gets the current waypoint (position).
	 * 
	 * @return the current waypoint (position)
	 */
	protected RLWaypoint getCurrentWaypoint() {
		return this.currentWp;
	}
	
	/**
	 * Sets the current waypoint (position)
	 * 
	 * @param the current waypoint (position)
	 */
	protected void setCurrentWaypoint(RLWaypoint wp) {
		this.currentWp = wp;
	}
	
	/**
	 * Gets the next state.
	 * 
	 * @return the next state
	 */
	protected State getNextState() {
		return this.nextState;
	}
	
	/**
	 * Sets the next state of this planner.
	 * 
	 * @param nextState the next state of this planner
	 */
	protected void setNextState(State nextState) {
		this.nextState = nextState;
	}
	
	/**
	 * Gets the next waypoint (position).
	 * 
	 * @return the next waypoint (position)
	 */
	protected RLWaypoint getNextWaypoint() {
		return this.nextWp;
	}
	
	/**
	 * Sets the next waypoint (position)
	 * 
	 * @param the next waypoint (position)
	 */
	protected void setNextWaypoint(RLWaypoint wp) {
		this.nextWp = wp;
	}
	
	/**
	 * Gets the plan's ETD.
	 * 
	 * @return the plan's ETD
	 */
	protected ZonedDateTime getEtd() {
		return this.etd;
	}
	
	/**
	 * Sets the plan's ETD.
	 * 
	 * @param etd this plan's ETD
	 */
	protected void setEtd(ZonedDateTime etd) {
		this.etd = etd;
	}
	
	/**
	 * Gets the chosen action index.
	 * 
	 * @return the chosen action index
	 */
	protected Action getAction() {
		return this.action;
	}
	
	/**
	 * Sets the chosen action index.
	 * 
	 * @param action the chosen action index
	 */
	protected void setAction(Action action) {
		this.action = action;
	}
	
	/**
	 * Gets the reward.
	 * 
	 * @return the reward
	 */
	protected double getReward() {
		return this.reward;
	}
	
	/**
	 * Sets the reward.
	 * 
	 * @param d the reward
	 */
	protected void setReward(double d) {
		this.reward = d;
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
	 * Gets the number of training episodes of this  planner.
	 * 
	 * @return the number of training episodes of this planner
	 */
	protected int getMaxEpisodes() {
		return this.maxEpisodes;
	}
	
	/**
	 * Sets the number of training episodes of this  planner.
	 * 
	 * @param max the number of training episodes of this planner
	 */
	protected void setMaxEpisodes(int max) {
		this.maxEpisodes = max;
	}
	
	/**
	 * Gets the max number of steps during training and computing of this  planner.
	 * 
	 * @return the max number of steps during training and computing of this planner
	 */
	protected int getMaxSteps() {
		return this.maxSteps;
	}
	
	/**
	 * Sets the max number of steps during training and computing of this  planner.
	 * 
	 * @param max max number of steps during training and computing of this planner
	 */
	protected void setMaxSteps(int max) {
		this.maxSteps = max;
	}
	
	/**
	 * Creates a waypoint at a specified position
	 * 
	 * @param position the position
	 * 
	 * @return the waypoint at the specified position
	 */
	protected RLWaypoint createWaypoint(Position position) {
		RLWaypoint wp = new RLWaypoint(position);
		
		Optional<? extends RLWaypoint> discoveredWaypoint = this.findDiscovered(wp);
		if (discoveredWaypoint.isPresent()) {
			wp = discoveredWaypoint.get();
		} else {
			wp.setDesignator(String.valueOf(wpId));
			wp.setCost(0);
			// If this is goal, send wp position twice
			if (this.getGoal()==null) {
				//State state = new State(wp.getPrecisionPosition(), wp, this.getEnvironment().getGlobe(), this.stateId);
			} else {
				//State state = new State(wp.getPrecisionPosition(), this.getGoal(), this.getEnvironment().getGlobe(), this.stateId);
			}
			if (this.qTable.containsKey(state)) {
				state = this.qTable.get(state).getState();
			}
			this.stateId++;
			wpId++;
			addDiscovered(wp);
		} 
		return wp;
	}
	
//	/**
//	 * Gets the neighbors of an RL Waypoint in the environment.
//	 * 
//	 * @param waypoint the RL waypoint 
//	 * 
//	 * @return the neighbors of the RL waypoint
//	 */
//	protected Set<RLWaypoint> getNeighbors(RLWaypoint waypoint) {
//		Set<Position> neighbors = this.getEnvironment().getNeighbors(waypoint);
//
//		// if a start has no neighbors, then it is not a waypoint in the
//		// environment and its adjacent waypoints have to be determined for
//		// initial expansion
//		if (neighbors.isEmpty()) {
//			neighbors = this.getEnvironment().getAdjacentWaypointPositions(waypoint);
//		}
//		
//		// create neighborhood of waypoints with precision positions
//		Set<RLWaypoint> neighborhood =
//				neighbors.stream()
//				.map(n -> this.createWaypoint(n)).collect(Collectors.toSet());
//		
//		// replace start and goal in neighborhood to avoid duplication
//		if (neighborhood.remove(this.getStart())) {
//			neighborhood.add(this.getStart());
//		}
//		if (neighborhood.remove(this.getGoal())) {
//			neighborhood.add(this.getGoal());
//		}
//
//		// expand start region position towards the start
//		if ((!waypoint.equals(this.getStart())) &&
//				this.isInStartRegion(waypoint.getPrecisionPosition())) {
//			// TODO: possible feasibility / capability issues
//			neighborhood.add(this.getStart());
//		}
//
//		// expand a goal region position towards the goal
//		if ((!waypoint.equals(this.getGoal())) &&
//				this.isInGoalRegion(waypoint.getPrecisionPosition())) {
//			// TODO: possible feasibility / capability issues
//			neighborhood.add(this.getGoal());
//		}
//		
//		return neighborhood;
//	}
	
	
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

		// Goal is created first to calculate start state when initializing the start waypoint
		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setPoi(true);
		
		this.setStart(this.createWaypoint(origin));
		this.getStart().setEto(etd);
		this.getStart().setPoi(true);
	
		
		// creates the start and goal states, and adds them to the Q table
		//this.getStart().setState(new State(this.getStart().getPrecisionPosition(), this.getGoal(), this.getEnvironment().getGlobe(), this.stateId));
		this.stateId ++;
		// this.getGoal().setState(new State(this.getGoal().getPrecisionPosition(), this.getGoal(), this.getEnvironment().getGlobe(), this.stateId));
		this.stateId ++;
		this.addQTable(this.getStart().getState());
		this.addQTable(this.getGoal().getState());
	}
	
	/**
	 * Initializes this Q-Learning planner to plan from an origin to a destination, 
	 * along waypoints, at a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void initialize(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
//		this.setStart(null);
//		this.setGoal(null);
//		clearInterWaypoints();
//
//		this.setStart(this.createWaypoint(origin));
//		this.getStart().setEto(etd);
//		this.getStart().setPoi(true);
//		
//		this.setGoal(this.createWaypoint(destination));
//		this.getGoal().setPoi(true);
//		
//		// creates the start and goal states, and adds them to the Q table
//		this.getStart().setState(new State(this.getStart().getPrecisionPosition(), this.getGoal(), this.getEnvironment().getGlobe(), this.stateId));
//		this.stateId ++;
//		this.getGoal().setState(new State(this.getGoal().getPrecisionPosition(), this.getGoal(), this.getEnvironment().getGlobe(), this.stateId));
//		this.stateId ++;
//		this.addQTable(this.getStart().getState());
//		this.addQTable(this.getGoal().getState());
//		
//		RLWaypoint wp;
//		for (Position position : waypoints) {
//			wp = this.createWaypoint(position);
//			wp.setState(new State(wp.getPrecisionPosition(), this.getGoal(), this.getEnvironment().getGlobe(), this.stateId));
//			this.stateId ++;
//			this.addInterWaypoint(wp);
//			this.addQTable(wp.getState());
//		}
//		
//		this.interWaypoints.remove(this.getStart());
//		this.addInterWaypoint(this.getGoal());
//		
//		// the adjacent waypoints to the origin
//		this.setStartRegion(this.getEnvironment().getAdjacentWaypointPositions(origin)
//				.stream()
//				.map(PrecisionPosition::new)
//				.collect(Collectors.toSet()));
//		
//		// the adjacent waypoints to the destination
//		this.setGoalRegion(this.getEnvironment().getAdjacentWaypointPositions(destination)
//				.stream()
//				.map(PrecisionPosition::new)
//				.collect(Collectors.toSet()));
	}
	
	/**
	 * Gets waypoint's neighbors and adds it as a state to the Q-table
	 * 
	 * @param wp the waypoint
	 */
	protected void addQTable(State state) {
		QLine newLine =  new QLine(state);
		newLine.initQValues();
		//qTable.put(state.getId(), newLine);
	}
	
	/**
	 * Computes the estimated ETO a specified source RL Waypoint when travelling
	 * from a source RL Waypoint
	 * 
	 * @param source the source RL waypoint in globe coordinates
	 * @param target the target RL waypoint in globe coordinates
	 * 
	 * @return the target's ETO
	 */
	protected ZonedDateTime computeTime(RLWaypoint source, RLWaypoint target) {
		Path leg = new Path(source.getPrecisionPosition().getOriginal(),
				target.getPrecisionPosition().getOriginal());
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		
		return end;
	}
	
	/**
	 * Calculates the reward based on the next state (only takes into account 
	 * distance to goal for now)
	 */
	public void calculateReward() {
//		// If it has reached the goal region, the reward is 100
//		if (isInGoalRegion(this.nextState.getPrecisionPosition())) {
//			setReward(1000);
//			return;
//		}
//		
//		// If it exceeds risk, the reward is -1000
//		if (this.nextState.hasInfiniteCost()) {
//			setReward(-1000);
//			return;
//		}
//		
//		ZonedDateTime end1 = computeTime(this.state, this.goal);
//		ZonedDateTime end2 = computeTime(this.nextState, this.goal);
//		
//		// If goal's ETO is higher for the nextState then for state, reward is -10, else it is 0
//		if (end2.isAfter(end1)) {
//			long amount = end1.until(end2, ChronoUnit.NANOS);
//			//setReward(-1 -0.000001*amount - this.nextState.getCost());
//			setReward(-10-0.000001*amount);
//		} else {
//			long amount = end2.until(end1, ChronoUnit.NANOS);
//			//setReward(-1 -this.nextState.getCost());
//			setReward(-10+0.00001*amount);
//		}
	}
	
	/**
	 * Calculates the reward based on the next state (only takes into account 
	 * distance to goal for now)
	 */
	public void calculateReward(RLWaypoint goal) {
//		// If it has reached the goal or intermediate waypoint region, the reward is 100
//		if (isInWaypointRegion(this.nextState.getPrecisionPosition(), goal)) {
//			setReward(1000);
//			return;
//		}
//		
//		// If it exceeds risk, the reward is -1000
//		if (this.nextState.hasInfiniteCost()) {
//			setReward(-1000);
//			return;
//		}
//		
//		ZonedDateTime end1 = computeTime(this.state, goal);
//		ZonedDateTime end2 = computeTime(this.nextState, goal);
//		
//		// If goal's ETO is higher for the nextState then for state, reward is -10, else it is 0
//		if (end2.isAfter(end1)) {
//			long amount = end1.until(end2, ChronoUnit.NANOS);
//			//setReward(-1 -0.000001*amount - this.nextState.getCost());
//			setReward(-10-0.000001*amount);
//		} else {
//			long amount = end2.until(end1, ChronoUnit.NANOS);
//			//setReward(-1 -this.nextState.getCost());
//			setReward(-10+0.00001*amount);
//		}
	}
	
	/**
	 * Calculates the new Q value based on the Q-Learning equation:
	 * newQ(s,a) = Q(s,a) + alpha.( R(s,a) + gamma.max(Q(s',a') - Q(s,a))
	 * and updates the table
	 */
	public void updateQ() {
//		// Calculates new Q value for state and action pair
//		double newQ = (1-this.alpha)*qTable.get(this.state.getDesignator()).getQValue(this.action) + this.alpha 
//				* (this.reward + this.gamma *qTable.get(this.nextState.getDesignator()).getMaxQValue());
//		// Updates the corresponding Q-table entry
//		qTable.get(this.state.getDesignator()).updateQValue(this.action, newQ);
	}
	
	/**
	 * Runs the training to learn the Q-values, for a single part trajectory
	 */
	public void singleTrain() {
//		// Loop to run all the training episodes
//		RLWaypoint ns = start;
//		int ep = 0;
//		while (ep < this.maxEpisodes) {
//			// Sets all waypoints' costs to zero
//			for (RLWaypoint wp : discovered) {
//				wp.setCost(0);
//			}
//			setState(this.getStart());
//			// Loop to go from start to goal
//			for (int j = 0; j < this.maxSteps; j++) {
//				// Chooses action according to e-greedy policy
//				setAction(this.qTable.get(this.state.getDesignator()).getEGreedyAction(this.epsilon));
//				// Updates next state
//				ns = createWaypoint(this.state.getNeighbor(getAction()));
//				computeEto(this.state, ns);
//				this.computeCost(this.state, ns);
//				setNextState(ns);
//				// Adds next state to Q-table if it isn't there yet
//				addQTable(this.nextState);
//				// Gets reward
//				calculateReward();
//				// Updates Q-Table
//				updateQ();
//				// Checks if it reached goal and, if yes, finish loop and increments ep
//				if (isInGoalRegion(this.nextState.getPrecisionPosition())) {
//					break;
//				}
//				setState(ns);
//			}
//			ep++;
//			// Updates epsilon for next episode
//			this.epsilon = this.minEpsilon + (this.maxEpsilon - this.minEpsilon)*Math.exp(-(2/this.maxEpisodes)*this.epsilon);
//			// Sets waypoints cost back to 0 and visited to false for next episode
//			for (Map.Entry<String, QLine> entry : qTable.entrySet()) {
//				entry.getValue().getState().setCost(0);
//				entry.getValue().getState().setVisited(false);
//			}
//		}
		
	}
	
	/**
	 * Runs the training to learn the Q-values, for a multi-part trajectory
	 */
	public void multiTrain() {
//		// Loop to run all the training episodes
//		RLWaypoint ns = start;
//		int ep = 0;
//		while (ep < this.maxEpisodes) {
//			// Sets all waypoints' costs to zero
//			for (RLWaypoint wp : discovered) {
//				wp.setCost(0);
//			}
//			setState(this.getStart());
//			for (RLWaypoint inter : this.interWaypoints)
//				// Loop to go from one waypoint to the next
//				for (int j = 0; j < this.maxSteps; j++) {
//					// Chooses action according to e-greedy policy
//					setAction(this.qTable.get(this.state.getDesignator()).getEGreedyAction(this.epsilon));
//					// Updates next state
//					ns = createWaypoint(this.state.getNeighbor(getAction()));
//					computeEto(this.state, ns);
//					this.computeCost(this.state, ns);
//					setNextState(ns);
//					// Adds next state to Q-table if it isn't there yet
//					addQTable(this.nextState);
//					// Gets reward
//					calculateReward(inter);
//					// Updates Q-Table
//					updateQ();
//					// System.out.println("State: " + this.state.getId() + " Action: " + this.action + " Reward: " + this.reward + " Next State:" + this.nextState.getId());
//					// Checks if it reached goal and, if yes, finish loop and increments ep
//					if (isInWaypointRegion(this.nextState.getPrecisionPosition(), inter)) {
//						break;
//					}
//					setState(ns);
//				}
//			ep++;
//			// Updates epsilon for next episode
//			this.epsilon = this.minEpsilon + (this.maxEpsilon - this.minEpsilon)*Math.exp(-(2/this.maxEpisodes)*this.epsilon);
//			// Sets waypoints cost back to 0 and visited to false for next episode
//			for (Map.Entry<String, QLine> entry : qTable.entrySet()) {
//				entry.getValue().getState().setCost(0);
//				entry.getValue().getState().setVisited(false);
//			}
//		}
//		
	}
	
	/**
	 * Computes a plan according to the trained Q-Table.
	 */
	protected void compute() {
//		System.out.println("FINAL PLAN from " + this.start.getDesignator() + " to " + this.goal.getDesignator());
//		//this.clearWaypoints();
//		// Only adds start to plan if it is not already there from the previous part
//		if (this.getWaypoints().isEmpty()) {
//			this.getWaypoints().addLast(this.start);
//		}
//		if(!(this.getWaypoints().getLast().equals(this.start))){
//			this.getWaypoints().addLast(this.start);
//		}
//		System.out.println("State: " + this.getWaypoints().getLast().getDesignator());
//		// Sets all waypoints to not visited yet and costs to zero
//		for (RLWaypoint wp : discovered) {
//			wp.setCost(0);
//			wp.setVisited(false);
//		}
//		RLWaypoint ns = start;
//		setState(this.getStart());
//		state.setVisited(true);
//		int steps = 0;
//		while (steps<this.maxSteps) {
//			// Chooses action according to Q-table and avoids states already in the plan or with infinite costs
//			int a = 1;
//			boolean notInfinite = false;
//			while((ns.isVisited() || ns.hasInfiniteCost())  && a<this.state.getNeighbors().size()+1) {
//				setAction(this.qTable.get(this.state.getDesignator()).getMaxQValue(a));
//				// Updates next state
//				ns = createWaypoint(this.state.getNeighbor(getAction()));
//				computeEto(this.state, ns);
//				this.computeCost(this.state, ns);
//				if (!ns.hasInfiniteCost()) notInfinite = true;
//				a++;
//			}
//			// If there is no neighbor that doesn't have infinite cost it stops computing and returns no trajectory
//			if (!notInfinite) {
//				System.out.println("No neighbors");
//				this.clearWaypoints();
//				break;
//			}
//			// If all neighbors have already been visited, then it chooses the one with max Q value
//			if (ns.isVisited()) {
//				setAction(this.qTable.get(this.state.getDesignator()).getMaxQValue());
//				// Updates next state
//				ns = createWaypoint(this.state.getNeighbor(getAction()));
//				computeEto(this.state, ns);
//				this.computeCost(this.state, ns);
//			}
//			ns.setVisited(true);
//			this.getWaypoints().addLast(ns);
//			System.out.println("State: " + this.getWaypoints().getLast().getDesignator());
//			setNextState(ns);
////			// Adds next state to Q-table if it isn't there yet
////			addQTable(this.nextState);
////			// Gets reward
////			calculateReward();
////			// Updates Q-Table
////			updateQ();
//			// Checks if it reached goal. If yes, recompute ETOs to make sure they are right and finish loop
//			if (isInGoalRegion(this.nextState.getPrecisionPosition())) {
//				if(!(this.getWaypoints().getLast().equals(this.goal))){
//					this.getWaypoints().addLast(this.goal);
//					System.out.println("State: " + this.getWaypoints().getLast().getDesignator());
//				}
//				this.getWaypoints().getFirst().setEto(getEtd());
//				for (int i=1; i< this.getWaypoints().size(); ++i) {
//					computeEto(this.getWaypoints().get(i-1), this.getWaypoints().get(i));
//				}
//				break;
//			}
//			setState(ns);
//			steps++;
//		}
//		// If it reached max number of steps without reaching goal, clears trajectory
////		if(!this.getWaypoints().isEmpty()) {
////			if(!(this.getWaypoints().getLast().equals(this.goal))){
////				System.out.println("Reached max steps");
////				this.clearWaypoints();
////			}
////		}
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
		this.clearWaypoints();
		this.clearDiscovered();
		setEtd(etd);
		this.qTable.clear();
		this.initialize(origin, destination, etd);
		this.singleTrain();
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
		this.clearWaypoints();
		this.clearDiscovered();
		setEtd(etd);
		RLWaypoint currentOrigin = new RLWaypoint(origin);
		ZonedDateTime currentEtd = etd;
		
		this.qTable.clear();
		this.initialize(origin, destination, waypoints, currentEtd);
		this.multiTrain();
		
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
			supports = (environment instanceof PlanningContinuum);
		}
		
		return supports;
	}
	
}

