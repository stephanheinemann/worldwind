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
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.environments.PlanningRoadmap;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;
import com.cfar.swim.worldwind.planners.rl.Plot;
import com.cfar.swim.worldwind.planners.rl.QLine;
import com.cfar.swim.worldwind.planners.rl.RLWaypoint;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
//import com.cfar.swim.worldwind.tests.Plot;
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
	protected Map<String, QLine> qTable = new TreeMap<>();
	
	/** the set of already discovered waypoints */
	protected Set<RLWaypoint> discovered = new HashSet<>();
	
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
	private double reward = 0;
	
	/** the start region */
	private Set<PrecisionPosition> startRegion;
	
	/** the goal region */
	private Set<PrecisionPosition> goalRegion;
	
	/** the maximum number of episodes in the learning phase of this planner */
	private int maxEpisodes = 500; 
	
	/** the maximum number of steps from start to goal of this planner */
	private int maxSteps = 50; 
	
	/** the maximum number of steps from start to goal of this planner */
	private double alpha = 0.8; 
	
	/** the minimum, maximum and current epsilon value to implement decaying e-greedy */
	private double minEpsilon = 0.01; 
	private double maxEpsilon = 1; 
	private double epsilon = 1; 
	
	/** the maximum number of steps from start to goal of this planner */
	private double gamma = 0.9; 
	
	/** waypoint counter */
	private int wpId = 0;
	
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
	 * Clears the discovered waypoints set
	 */
	protected void clearDiscovered() {
		this.discovered.clear();
		wpId = 0;
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
			wpId++;
			discovered.add(wp);
		} 
		return wp;
	}
	
	/**
	 * Gets the neighbors of an RL Waypoint in the environment.
	 * 
	 * @param waypoint the RL waypoint 
	 * 
	 * @return the neighbors of the RL waypoint
	 */
	protected Set<RLWaypoint> getNeighbors(RLWaypoint waypoint) {
		Set<Position> neighbors = this.getEnvironment().getNeighbors(waypoint);

		// if a start has no neighbors, then it is not a waypoint in the
		// environment and its adjacent waypoints have to be determined for
		// initial expansion
		if (neighbors.isEmpty()) {
			neighbors = this.getEnvironment().getAdjacentWaypointPositions(waypoint);
		}
		
		// create neighborhood of waypoints with precision positions
		Set<RLWaypoint> neighborhood =
				neighbors.stream()
				.map(n -> this.createWaypoint(n)).collect(Collectors.toSet());
		
		// replace start and goal in neighborhood to avoid duplication
		if (neighborhood.remove(this.getStart())) {
			neighborhood.add(this.getStart());
		}
		if (neighborhood.remove(this.getGoal())) {
			neighborhood.add(this.getGoal());
		}

		// expand start region position towards the start
		if ((!waypoint.equals(this.getStart())) &&
				this.isInStartRegion(waypoint.getPrecisionPosition())) {
			// TODO: possible feasibility / capability issues
			neighborhood.add(this.getStart());
		}

		// expand a goal region position towards the goal
		if ((!waypoint.equals(this.getGoal())) &&
				this.isInGoalRegion(waypoint.getPrecisionPosition())) {
			// TODO: possible feasibility / capability issues
			neighborhood.add(this.getGoal());
		}
		
		return neighborhood;
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
		this.qTable.clear();
		
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
		
		this.addQTable(this.getStart());
		this.addQTable(this.getGoal());
	}
	
	/**
	 * Gets waypoint's neighbors and adds it as a state to the Q-table
	 * 
	 * @param wp the waypoint
	 */
	protected void addQTable(RLWaypoint wp) {
		// only adds new waypoint to the Q-table if it is not there yet
		if (!(qTable.containsKey(wp.getDesignator()))) {
			Set<RLWaypoint> neighbors = getNeighbors(wp);
			wp.setNeighbors(neighbors);
			QLine newLine =  new QLine(wp, neighbors.size());
			newLine.initQValues();
			qTable.put(wp.getDesignator(), newLine);
		} else {
		// if waypoint already exists in qTable, set new waypoint to that
			wp = qTable.get(wp.getDesignator()).getState();
		}
	}
	
//	/**
//	 * Computes the estimated ETO of a specified source RL Waypoint when travelling
//	 * from a source RL Waypoint
//	 * 
//	 * @param source the source RL waypoint in globe coordinates
//	 * @param target the target RL waypoint in globe coordinates
//	 */
//	protected double computeCost(RLWaypoint source, RLWaypoint target) {
//		Path leg = new Path(source, target);
//		Capabilities capabilities = this.getAircraft().getCapabilities();
//		Globe globe = this.getEnvironment().getGlobe();
//		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
//		double cost = this.getEnvironment().getStepCost(
//				source, target,
//				source.getEto(), end,
//				this.getCostPolicy(), this.getRiskPolicy());
//		
//		return cost;
//	}
	
	/**
	 * Computes the estimated costof a specified source RL Waypoint when travelling
	 * from a source RL Waypoint
	 * 
	 * @param source the source RL waypoint in globe coordinates
	 * @param target the target RL waypoint in globe coordinates
	 */
	protected ZonedDateTime computeEto(RLWaypoint source, RLWaypoint target) {
		// TODO: align with AbstractPlanner (computeEto, computeCost)
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch CapabilitiesException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		
		return end;
	}
	
	/**
	 * Calculates the reward based on the next state (only takes into account 
	 * distance to goal for now)
	 */
	public void calculateReward() {
		// If it has reached the goal region, the reward is 100
		if (this.nextState.equals(this.goal)) {
			setReward(100);
			return;
		}
		
		// If it exceeds risk, the reward is -100
		if (this.nextState.hasInfiniteCost()) {
			setReward(-1000);
			return;
		}
		
		ZonedDateTime end1 = computeEto(this.state, this.goal);
		ZonedDateTime end2 = computeEto(this.nextState, this.goal);
		
		// If goal's ETO is higher for the nextState then for state, reward is -10, else it is 0
		if (end2.isAfter(end1)) {
			long amount = end1.until(end2, ChronoUnit.NANOS);
			setReward(-0.000001*amount - this.nextState.getCost());
			//setReward(-0.000001*amount);
		} else {
			setReward(-this.nextState.getCost());
			//setReward(0);
		}
	}
	
	/**
	 * Calculates the new Q value based on the Q-Learning equation:
	 * newQ(s,a) = Q(s,a) + alpha.( R(s,a) + gamma.max(Q(s',a') - Q(s,a))
	 * and updates the table
	 */
	public void updateQ() {
		// Calculates new Q value for state and action pair
		double newQ = (1-this.alpha)*qTable.get(this.state.getDesignator()).getQValue(this.action) + this.alpha 
				* (this.gamma * (this.reward + qTable.get(this.nextState.getDesignator()).getMaxQValue()));
		// Updates the corresponding Q-table entry
		qTable.get(this.state.getDesignator()).updateQValue(this.action, newQ);
	}
	
	/**
	 * Runs the training to learn the Q-values
	 */
	public void train() {
		// Loop to run all the training episodes
		RLWaypoint ns = start;
		int ep = 0;
		while (ep < this.maxEpisodes) {
			// System.out.println("EPISODE " + ep);
			setState(this.getStart());
			// Loop to go from start to goal
			for (int j = 0; j < this.maxSteps; j++) {
				// Chooses action according to e-greedy policy
				setAction(this.qTable.get(this.state.getDesignator()).getEGreedyAction(this.epsilon));
				// Updates next state
				ns = createWaypoint(this.state.getNeighbor(getAction()));
				ns.setEto(computeEto(this.state, ns));
				this.computeCost(this.state, ns);
				setNextState(ns);
				// Adds next state to Q-table if it isn't there yet
				addQTable(this.nextState);
				// Gets reward
				calculateReward();
				// Updates Q-Table
				updateQ();
				// System.out.println("State: " + this.state.getId() + " Action: " + this.action + " Reward: " + this.reward + " Next State:" + this.nextState.getId());
				// Checks if it reached goal and, if yes, finish loop and increments ep
				if (isInGoalRegion(this.nextState.getPrecisionPosition())) {
					break;
				}
				setState(ns);
			}
			ep++;
			// Updates epsilon for next episode
			this.epsilon = this.minEpsilon + (this.maxEpsilon - this.minEpsilon)*Math.exp(-(1/this.maxEpisodes)*this.epsilon);
			// Sets waypoints cost back to 0 and visited to false for next episode
			for (Map.Entry<String, QLine> entry : qTable.entrySet()) {
				entry.getValue().getState().setCost(0);
				entry.getValue().getState().setVisited(false);
			}
		}
		
	}
	
	/**
	 * Computes a plan according to the trained Q-Table.
	 */
	protected void compute() {
		//this.clearWaypoints();
		// Only adds start to plan if it is not already there from the previous part
		if (this.getWaypoints().isEmpty()) {
			this.getWaypoints().addLast(this.start);
		}
		if(!(this.getWaypoints().getLast().equals(this.start))){
			this.getWaypoints().addLast(this.start);
		}
		RLWaypoint ns = start;
		setState(this.getStart());
		state.setVisited(true);
		int steps = 0;
		// Prints to console the final Q-table before compute
		System.out.println("Q-TABLE");
		String line;
		for (Map.Entry<String, QLine> entry : qTable.entrySet()) {
			line = "State: " + String.valueOf(entry.getValue().getState().getDesignator()) + " Values:";
			for(int i=0; i<entry.getValue().getActions(); i++) {
				line = line + " " + entry.getValue().getState().getNeighbor(i).getDesignator() + "-" + String.valueOf(entry.getValue().getQValue(i));
			}
			System.out.println(line);
		}
		System.out.println("FINAL PLAN ");
		while (steps<this.maxSteps) {
			// Chooses action according to Q-table and avoids states already in the plan or with infinite costs
			int a = 1;
			boolean notInfinite = false;
			while((ns.isVisited() || ns.hasInfiniteCost())  && a<this.state.getNeighbors().size()+1) {
				setAction(this.qTable.get(this.state.getDesignator()).getMaxQValue(a));
				// Updates next state
				ns = createWaypoint(this.state.getNeighbor(getAction()));
				this.computeCost(this.state, ns);
				if (!ns.hasInfiniteCost()) notInfinite = true;
				a++;
			}
			// If there is no neighbor that doesn't have infinite cost it stops computing and returns no trajectory
			if (!notInfinite) {
				this.clearWaypoints();
				break;
			}
			// If all neighbors have already been visited, then it chooses the one with max Q value
			if (ns.isVisited()) {
				setAction(this.qTable.get(this.state.getDesignator()).getMaxQValue());
				// Updates next state
				ns = createWaypoint(this.state.getNeighbor(getAction()));
			}
			ns.setVisited(true);
			ns.setEto(computeEto(this.state, ns));
//			this.computeCost(this.state, ns);
			this.getWaypoints().addLast(ns);
			setNextState(ns);
			// Adds next state to Q-table if it isn't there yet
			addQTable(this.nextState);
			// Gets reward
			calculateReward();
			// Updates Q-Table
			updateQ();
			// Checks if it reached goal and, if yes, finish loop
			if (isInGoalRegion(this.nextState.getPrecisionPosition())) {
				if(!(this.getWaypoints().getLast().equals(this.goal))){
					goal.setEto(computeEto(this.nextState, goal));
					this.getWaypoints().addLast(this.goal);
				}
				break;
			}
			setState(ns);
			steps++;
		}
		for(int i=0; i<this.getWaypoints().size(); i++) {
			System.out.println("State: " + this.getWaypoints().get(i).getDesignator() + " ETO: " + this.getWaypoints().get(i).getEto());
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
//		// Starts plot
//		Plot plot = new Plot("Hey", 88.1470000, 88.153000, 0.000001, 44.900000, 45.1000000, 0.00001);
//		plot.setPointSize(6);
//		plot.setPointShape(Plot.CIRCLE);
//		plot.setBackground(Color.white);
//		// Prints start in red
//		plot.setColor(Color.red);
//		plot.addPoint(start.latitude.degrees, start.longitude.degrees, String.valueOf(start.getId()));
////		// Prints start's neighbors in red
////		Iterator<RLWaypoint> neighborsIterator = this.start.getNeighbors().iterator();
////		while (neighborsIterator.hasNext()) {
////			RLWaypoint p = neighborsIterator.next();
////			plot.addPoint(p.latitude.degrees, p.longitude.degrees, String.valueOf(p.getId()));
////		}
//		// Prints goal in green
//		plot.setColor(Color.green);
//		plot.addPoint(goal.latitude.degrees, goal.longitude.degrees, String.valueOf(goal.getId()));
////		// Prints goal's neighbors in green
////		neighborsIterator = this.goal.getNeighbors().iterator();
////		while (neighborsIterator.hasNext()) {
////			RLWaypoint p = neighborsIterator.next();
////			plot.addPoint(p.latitude.degrees, p.longitude.degrees, String.valueOf(p.getId()));
////		}
		this.compute();
		// Prints all discovered points in cyan
//		plot.setColor(Color.cyan);
//		for (Map.Entry<RLWaypoint, QLine> entry : qTable.entrySet()) {
//			plot.addPoint(entry.getKey().latitude.degrees, entry.getKey().longitude.degrees, String.valueOf(entry.getKey().getId()));
//		}
//		// Prints final plan in black
//		plot.setColor(Color.black);
//		for (int i=0; i<this.getPlan().size(); i++) {
//			plot.setConnected(true);
//			plot.addPoint(this.getPlan().get(i).latitude.degrees, this.getPlan().get(i).longitude.degrees, "");
//		}
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
		this.initialize(origin, destination, etd);
		this.train();
		Trajectory trajectory = this.planPart(0);
		this.revisePlan(trajectory);
//		// Starts plot
//		Plot plot = new Plot("Hey", 88.1470000, 88.153000, 0.000001, 44.900000, 45.1000000, 0.00001);
//		plot.setPointSize(6);
//		plot.setPointShape(Plot.CIRCLE);
//		plot.setBackground(Color.white);
//		// Prints all discovered points in cyan
//		plot.setColor(Color.cyan);
//		for (Map.Entry<String, QLine> entry : qTable.entrySet()) {
//			plot.addPoint(entry.getValue().getState().latitude.degrees, entry.getValue().getState().longitude.degrees, "");
//			System.out.println("Cost of point " + entry.getKey() + ": " + entry.getValue().getState().getCost());
//		}
//		// Prints final plan in black
//		plot.setColor(Color.black);
//		for (int i=0; i<this.getPlan().size(); i++) {
//			plot.addPoint(this.getPlan().get(i).latitude.degrees, this.getPlan().get(i).longitude.degrees, "");
//			plot.setConnected(true);
//		}
//		plot.setConnected(false);
//		// Prints start in red
//		plot.setColor(Color.red);
//		plot.addPoint(origin.latitude.degrees, origin.longitude.degrees, "");
//		// Prints goal in green
//		plot.setColor(Color.green);
//		plot.addPoint(destination.latitude.degrees, destination.longitude.degrees, "");
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
				this.train();
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

