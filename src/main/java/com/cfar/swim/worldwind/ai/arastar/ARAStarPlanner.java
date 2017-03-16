package com.cfar.swim.worldwind.ai.arastar;

import java.time.ZonedDateTime;
import java.util.HashSet;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;

import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.astar.AStarWaypoint;
import com.cfar.swim.worldwind.ai.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;

import gov.nasa.worldwind.geom.Position;


/**
 * Realizes an Anytime Replanning A* planner (ARA*) that plans a trajectory of
 * an aircraft in an environment considering a local cost and risk policy.
 * 
 * @author Stephan Heinemann
 *
 */
public class ARAStarPlanner extends ForwardAStarPlanner implements AnytimePlanner {

	/** the set of inconsistent already expanded waypoints */
	protected Set<ARAStarWaypoint> incons = new HashSet<>();
	
	/** the initial inflation factor applied to the heuristic function */
	private double initialInflation;
	
	/** the final inflation factor applied the heuristic function */
	private double finalInflation;
	
	/** the current inflation factor applied to the heuristic function */
	private double inflation;
	
	/** the deflation amount to be applied to the current inflation */
	private double deflationAmount;
	
	/**
	 * Constructs an ARA* planner for a specified aircraft and environment
	 * using default local cost and risk policies without an initial inflation 
	 * applied to the heuristic.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 */
	public ARAStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.setMinimumQuality(1d);
		this.setMaximumQuality(1d);
		this.setQualityImprovement(1d);
		this.setInflation(1d);
	}
	
	/**
	 * Gets the minimum quality (initial inflation) of this ARA* planner.
	 * 
	 * @return the minimum quality (initial inflation) of this ARA* planner
	 * 
	 * @see AnytimePlanner#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialInflation;
	}
	
	/**
	 * Sets the minimum quality (initial inflation) of this ARA* planner.
	 * 
	 * @param initialInflation the minimum quality (initial inflation) of this
	 *                         ARA* planner
	 * 
	 * @throws IllegalArgumentException if the initial inflation is invalid
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialInflation) {
		if ((1d <= initialInflation) &&
				(initialInflation >= this.finalInflation)) {
			this.initialInflation = initialInflation;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}
	
	/**
	 * Gets the maximum quality (final inflation) of this ARA* planner.
	 * 
	 * @return the maximum quality (final inflation) of this ARA* planner
	 * 
	 * @see AnytimePlanner#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalInflation;
	}
	
	/**
	 * Sets the maximum quality (initial inflation) of this ARA* planner.
	 * 
	 * @param finalInflation the maximum quality (final inflation) of this
	 *                       ARA* planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalInflation) {
		if ((1d <= finalInflation) && (this.initialInflation >= finalInflation)) {
			this.finalInflation = finalInflation;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}
	
	/**
	 * Gets the quality improvement (deflation amount) of this ARA* planner.
	 * 
	 * @return the quality improvement (deflation amount) of this ARA* planner
	 * 
	 * @see AnytimePlanner#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.deflationAmount;
	}
	
	/**
	 * Sets the quality improvement (deflation amount) of this ARA* planner.
	 * 
	 * @param deflationAmount the quality improvement (deflation amount) of
	 *                        this ARA* planner
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double deflationAmount) {
		if (0d < deflationAmount) {
			this.deflationAmount = deflationAmount;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}
	
	/**
	 * Deflates the current inflation by the deflation amount, or otherwise to
	 * the final deflation.
	 */
	protected void deflate() {
		if (this.finalInflation > (this.inflation - this.deflationAmount)) {
			this.inflation -= this.deflationAmount;
		} else {
			this.inflation = this.finalInflation;
		}
	}
	
	/**
	 * Determines whether or not the final deflation has been reached.
	 * 
	 * @return true if the final deflation has been reached, false otherwise
	 */
	protected boolean isDeflated() {
		return (this.inflation == this.finalInflation);
	}
	
	/**
	 * Gets the current inflation of this ARA* planner.
	 * 
	 * @return the current inflation of this ARA* planner
	 */
	protected double getInflation() {
		return this.inflation;
	}
	
	/**
	 * Sets the current inflation of this ARA* planner.
	 * 
	 * @param inflation the current inflation of this ARA* planner
	 */
	protected void setInflation(double inflation) {
		this.inflation = inflation;
	}
	
	/**
	 * Creates an ARA* waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the ARA* waypoint at the specified position
	 */
	@Override
	protected ARAStarWaypoint createWaypoint(Position position) {
		return new ARAStarWaypoint(position);
	}
	
	/**
	 * Gets the start ARA* waypoint of this ARA* planner.
	 * 
	 * @return the start ARA* waypoint of this ARA* planner
	 * 
	 * @see ForwardAStarPlanner#getStart()
	 */
	@Override
	protected ARAStarWaypoint getStart() {
		return (ARAStarWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal ARA* waypoint of this ARA* planner.
	 * 
	 * @return the goal ARA* waypoint of this ARA* planner
	 * 
	 * @see ForwardAStarPlanner#getGoal()
	 */
	@Override
	protected ARAStarWaypoint getGoal() {
		return (ARAStarWaypoint) super.getGoal();
	}
	
	/**
	 * Determines whether or not ARA* waypoints can be expanded.
	 * 
	 * @return true if ARA* waypoints can be expanded, false otherwise
	 * 
	 * @see ForwardAStarPlanner#canExpand()
	 */
	@Override
	protected boolean canExpand() {
		return super.canExpand() &&
				(this.getGoal().getF() > this.peekExpandable().getF());
	}
	
	/**
	 * Peeks an ARA* waypoint from the expandable ARA* waypoints.
	 * 
	 * @return the peeked ARA* waypoint from the expandable ARA* waypoints
	 *         if any, null otherwise
	 * 
	 * @see ForwardAStarPlanner#peekExpandable()
	 */
	@Override
	protected ARAStarWaypoint peekExpandable() {
		return (ARAStarWaypoint) super.peekExpandable();
	}
	
	/**
	 * Polls an ARA* waypoint from the expandable ARA* waypoints.
	 * 
	 * @return the polled ARA* waypoint from the expandable ARA* waypoints
	 *         if any, null otherwise
	 * 
	 * @see ForwardAStarPlanner#pollExpandable()
	 */
	@Override
	protected ARAStarWaypoint pollExpandable() {
		return (ARAStarWaypoint) super.pollExpandable();
	}
	
	/**
	 * Finds an expandable ARA* waypoint.
	 * 
	 * @param waypoint the expandable ARA* waypoint to be found
	 * 
	 * @return the found expandable ARA* waypoint if any
	 * 
	 * @see ForwardAStarPlanner#findExpandable(AStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Optional<? extends ARAStarWaypoint>
		findExpandable(AStarWaypoint waypoint) {
		return (Optional<ARAStarWaypoint>) super.findExpandable(waypoint);
	}
	
	/**
	 * Finds an expanded ARA* waypoint.
	 * 
	 * @param waypoint the expanded ARA* waypoint to be found
	 * 
	 * @return the found expanded ARA* waypoint, if any
	 * 
	 * @see ForwardAStarPlanner#findExpanded(AStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Optional<? extends ARAStarWaypoint>
		findExpanded(AStarWaypoint waypoint) {
		return (Optional<ARAStarWaypoint>) super.findExpanded(waypoint);
	}
	
	/**
	 * Expands an ARA* waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the ARA* waypoint to be expanded
	 * 
	 * @return the neighbors of the expanded ARA* waypoint
	 * 
	 * @see ForwardAStarPlanner#expand(AStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Set<? extends ARAStarWaypoint> expand(AStarWaypoint waypoint) {
		((ARAStarWaypoint) waypoint).makeConsistent();
		return (Set<ARAStarWaypoint>) super.expand(waypoint);
	}
	
	/**
	 * Adds an ARA* waypoint to the inconsistent ARA* waypoints.
	 * 
	 * @param waypoint the ARA* waypoint
	 * 
	 * @return true if the ARA* waypoint has been added to the inconsistent
	 *         ARA* waypoints, false otherwise
	 */
	protected boolean addInconsistent(ARAStarWaypoint waypoint) {
		return this.incons.add(waypoint);
	}
	
	/**
	 * Removes an ARA* waypoint from the inconsistent ARA* waypoints.
	 * 
	 * @param waypoint the ARA* waypoint
	 * 
	 * @return true if the ARA* waypoint has been removed from the inconsistent
	 *         ARA* waypoints, false otherwise
	 */
	protected boolean removeInconsistent(ARAStarWaypoint waypoint) {
		return this.incons.remove(waypoint);
	}
	
	/**
	 * Clears the inconsistent ARA* waypoints.
	 */
	protected void clearInconsistent() {
		this.incons.clear();
	}
	
	/**
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @see ForwardAStarPlanner#initialize(Position, Position, ZonedDateTime)
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		this.setInflation(this.getMinimumQuality());
		this.getGoal().setEpsilon(this.getInflation());
		this.getStart().setEpsilon(this.getInflation());
		this.clearInconsistent();
	}
	
	/**
	 * Updates the open set for an updated ARA* waypoint.
	 * 
	 * @param waypoint the updated ARA* waypoint
	 * 
	 * @see ForwardAStarPlanner#updateSets(AStarWaypoint)
	 */
	@Override
	protected void updateSets(AStarWaypoint waypoint) {
		Optional<? extends ARAStarWaypoint> expanded =
				this.findExpanded(waypoint);
		
		if (expanded.isPresent()) {
			this.addInconsistent(expanded.get());
		} else {
			super.updateSets(waypoint);
		}
	}
	
	/**
	 * Computes a plan.
	 * 
	 * @see ForwardAStarPlanner#compute()
	 */
	@Override
	protected void compute() {
		while (this.canExpand()) {
			ARAStarWaypoint source = this.pollExpandable();
			
			if (source.equals(this.getGoal())) {
				this.connectPlan(source);
				return;
			}
			
			Set<? extends ARAStarWaypoint> neighbors = this.expand(source);
			
			for (ARAStarWaypoint target : neighbors) {
				Optional<? extends ARAStarWaypoint> expandable =
						this.findExpandable(target);
				if (expandable.isPresent()) {
					target = expandable.get();
				} else {
					Optional<? extends ARAStarWaypoint> expanded =
							this.findExpanded(target);
					if (expanded.isPresent()) {
						target = expanded.get();
					}
				}
				this.updateWaypoint(source, target);
			}
		}
	}
	
	/*
	protected PriorityQueue<? extends ARAStarWaypoint> computeImprovement() {
		this.deflate();
		PriorityQueue<AStarWaypoint> updatedOpen = new PriorityQueue<>();
		this.open.stream().forEach(w -> {
			((ARAStarWaypoint) w).setEpsilon(this.getInflation());
			updatedOpen.add(w);
		});
		this.open = updatedOpen;
		this.incons.stream().forEach(w -> {
			((ARAStarWaypoint) w).setEpsilon(this.getInflation());
			this.addExpandable(w);
		});
		this.clearInconsistent();
		this.clearExpanded();
		this.compute();
	}
	*/
	
	protected void improve() {
		while (!this.isDeflated()) {
			
			this.deflate();
			PriorityQueue<AStarWaypoint> updatedOpen = new PriorityQueue<>();
			this.open.stream().forEach(w -> {
				((ARAStarWaypoint) w).setEpsilon(this.getInflation());
				updatedOpen.add(w);
			});
			this.open = updatedOpen;
			this.incons.stream().forEach(w -> {
				((ARAStarWaypoint) w).setEpsilon(this.getInflation());
				this.addExpandable(w);
			});
			this.clearInconsistent();
			this.clearExpanded();
			this.compute();
			Trajectory trajectory = this.createTrajectory();
			this.revisePlan(trajectory);
		}
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
	 * @see ForwardAStarPlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);
		this.compute();
		this.improve();
		Trajectory trajectory = this.createTrajectory();
		this.revisePlan(trajectory);
		return trajectory;
	}
	
	// TODO: compute all partial trajectories of one quality before improving
	// TODO: store new open for each partial iteration with one quality
	// TODO: updatedOpen is not a local variable, a list of open sets is required
	// TODO: ARA* could synchronize deflation!
}
