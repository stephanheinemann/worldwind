package com.cfar.swim.worldwind.ai.arastar;

import java.time.ZonedDateTime;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Set;

import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.astar.AStarWaypoint;
import com.cfar.swim.worldwind.ai.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

public class ARAStarPlanner extends ForwardAStarPlanner implements AnytimePlanner {

	/** the set of inconsistent already expanded waypoints */
	protected Set<ARAStarWaypoint> incons = new HashSet<>();
	
	private double initialInflation = 1d;
	private double finalInflation = 1d;
	private double deflationAmount = 1d;
	private double inflation = 1d;
	
	public ARAStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	@Override
	public double getMinimumQuality() {
		return this.initialInflation;
	}

	@Override
	public void setMinimumQuality(double initialInflation) {
		if (1d <= initialInflation) {
			this.initialInflation = initialInflation;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}
	
	@Override
	public double getMaximumQuality() {
		return this.finalInflation;
	}

	@Override
	public void setMaximumQuality(double finalInflation) {
		if ((1d <= finalInflation) && (this.initialInflation <= finalInflation)) {
			this.finalInflation = finalInflation;
		} else {
			throw new IllegalArgumentException("final defalation is invalid");
		}
	}
	
	@Override
	public double getQualityImprovement() {
		return this.deflationAmount;
	}

	@Override
	public void setQualityImprovement(double deflationAmount) {
		if (0d < deflationAmount) {
			this.deflationAmount = deflationAmount;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}
	
	protected void deflate() {
		if (this.finalInflation > (this.inflation - this.deflationAmount)) {
			this.inflation -= this.deflationAmount;
		} else {
			this.inflation = this.finalInflation;
		}
	}
	
	protected boolean isDeflated() {
		return (this.inflation == this.finalInflation);
	}
	
	@Override
	protected ARAStarWaypoint createWaypoint(Position position) {
		return new ARAStarWaypoint(position);
	}
	
	@Override
	protected ARAStarWaypoint getStart() {
		return (ARAStarWaypoint) super.getStart();
	}
	
	@Override
	protected ARAStarWaypoint getGoal() {
		return (ARAStarWaypoint) super.getGoal();
	}
	
	@Override
	protected boolean canExpand() {
		return super.canExpand() &&
				(this.getGoal().getF() > this.peekExpandable().getF());
	}
	
	@Override
	protected ARAStarWaypoint peekExpandable() {
		return (ARAStarWaypoint) super.peekExpandable();
	}
	
	@Override
	protected ARAStarWaypoint pollExpandable() {
		return (ARAStarWaypoint) super.pollExpandable();
	}
	
	@Override
	protected ARAStarWaypoint findExpandable(AStarWaypoint waypoint) {
		return (ARAStarWaypoint) super.findExpandable(waypoint);
	}
	
	@Override
	protected ARAStarWaypoint findExpanded(AStarWaypoint waypoint) {
		return (ARAStarWaypoint) super.findExpanded(waypoint);
	}
	
	@SuppressWarnings("unchecked")
	@Override
	protected Set<? extends ARAStarWaypoint> expand(AStarWaypoint waypoint) {
		return (Set<ARAStarWaypoint>) super.expand(waypoint);
	}
	
	//@Override
	protected void processSource(Waypoint source) {
		ARAStarWaypoint s = (ARAStarWaypoint) source;
		s.setV(s.getG());
	}
	
	//@Override
	protected void processTarget(Waypoint target) {
		ARAStarWaypoint t = (ARAStarWaypoint) target;
		t.setEpsilon(this.inflation);
	}
	
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		this.inflation = this.initialInflation;
		this.getGoal().setEpsilon(this.inflation);
		this.getStart().setEpsilon(this.inflation);
		this.incons.clear();
	}
	
	/**
	 * Computes a planned trajectory.
	 * 
	 * @see ForwardAStarPlanner#compute()
	 */
	@Override
	protected void compute() {
		while (this.canExpand()) {
			ARAStarWaypoint source = this.pollExpandable();
			this.processSource(source);
			if (source.equals(this.getGoal())) {
				this.connectPlan(source);
				return;
			}
			
			Set<? extends ARAStarWaypoint> neighbors = this.expand(source);
			
			for (ARAStarWaypoint target : neighbors) {
				// TODO: always compute cost first
				if (!this.isExpanded(target)) {
					if (this.isExpandable(target)) {
						ARAStarWaypoint visited = this.findExpandable(target);
						this.processTarget(visited);
						this.updateWaypoint(source, visited);
					} else {
						this.processTarget(target);
						this.updateWaypoint(source, target);
					}
				} else {
					ARAStarWaypoint expanded = this.findExpanded(target);
					// TODO: updating queues is wrong here
					this.updateWaypoint(source, expanded);
				}
			}
		}
	}
	
	protected void improve() {
		while (!this.isDeflated()) {
			this.deflate();
			PriorityQueue<AStarWaypoint> updatedOpen = new PriorityQueue<>();
			this.open.stream().forEach(w -> {
				((ARAStarWaypoint) w).setEpsilon(this.inflation);
				updatedOpen.add(w);
			});
			this.open = updatedOpen;
			this.incons.stream().forEach(w -> {
				((ARAStarWaypoint) w).setEpsilon(this.inflation);
				open.add(w);
			});
			this.incons.clear();
			this.closed.clear();
			this.compute();
		}
	}
	
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);
		this.compute();
		this.improve();
		return this.createTrajectory();
	}
	
}
