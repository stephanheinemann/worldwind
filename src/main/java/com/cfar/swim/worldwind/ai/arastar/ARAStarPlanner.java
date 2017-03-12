package com.cfar.swim.worldwind.ai.arastar;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.time.ZonedDateTime;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

public class ARAStarPlanner extends ForwardAStarPlanner implements AnytimePlanner {

	/** the set of inconsistent already expanded waypoints */
	protected Set<Waypoint> incons = new HashSet<Waypoint>();
	
	protected double inflationFactor = 1d;
	protected double deflationAmount = 1d;
	
	/** the property change support of this ARA* planner */
	protected final PropertyChangeSupport pcs = new PropertyChangeSupport(this);
	
	public ARAStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	@Override
	protected ARAStarWaypoint createWaypoint(Position position) {
		return new ARAStarWaypoint(position);
	}
	
	@Override
	protected ARAStarWaypoint getStart() {
		return (ARAStarWaypoint) this.start;
	}
	
	@Override
	protected ARAStarWaypoint getGoal() {
		return (ARAStarWaypoint) this.goal;
	}
	
	@Override
	protected boolean isExpandable() {
		return super.isExpandable() &&
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
	protected ARAStarWaypoint findExpandable(Waypoint waypoint) {
		return (ARAStarWaypoint) super.findExpandable(waypoint);
	}
	
	@Override
	protected ARAStarWaypoint findExpanded(Waypoint waypoint) {
		return (ARAStarWaypoint) super.findExpanded(waypoint);
	}
	
	//@Override
	protected void processSource(Waypoint source) {
		ARAStarWaypoint s = (ARAStarWaypoint) source;
		s.setV(s.getG());
	}
	
	//@Override
	protected void processTarget(Waypoint target) {
		ARAStarWaypoint t = (ARAStarWaypoint) target;
		t.setEpsilon(this.inflationFactor);
	}
	
	/**
	 * Connects a trajectory of specified waypoints.
	 * 
	 * @param waypoint the last waypoint of a computed trajectory
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected void connectTrajectory(Waypoint waypoint) {
		super.connectTrajectory(waypoint);
		// TODO: published waypoints may be modified later - clone each waypoint?
		this.pcs.firePropertyChange("plan", null, new Trajectory((List<Waypoint>) this.plan.clone()));
	}
	
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		this.getGoal().setEpsilon(this.inflationFactor);
		this.getStart().setEpsilon(this.inflationFactor);
		this.incons.clear();
	}
	
	/**
	 * Computes a planned trajectory.
	 */
	@Override
	protected void compute() {
		while (this.isExpandable()) {
			ARAStarWaypoint source = this.pollExpandable();
			this.processSource(source);
			if (source.equals(this.goal)) {
				this.connectTrajectory(source);
				return;
			}
			this.closed.add(source);
			
			Set<Position> neighbors = this.expand(source);
			
			for (Position neighbor : neighbors) {
				ARAStarWaypoint target = this.createWaypoint(neighbor);
				// TODO: always compute cost first
				if (!this.closed.contains(target)) {
					if (this.open.contains(target)) {
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
			PriorityQueue<Waypoint> updatedOpen = new PriorityQueue<>();
			this.open.stream().forEach(w ->
				{((ARAStarWaypoint) w).setEpsilon(this.inflationFactor); updatedOpen.add(w);});
			this.open = updatedOpen;
			this.incons.stream().forEach(w ->
				{((ARAStarWaypoint) w).setEpsilon(this.inflationFactor); updatedOpen.add(w);});
			this.incons.clear();
			this.closed.clear();
			this.compute();
		}
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);
		this.compute();
		this.improve();
		return new Trajectory((List<Waypoint>) this.plan.clone());
	}

	@Override
	public double getInflationFactor() {
		return this.inflationFactor;
	}

	@Override
	public void setInflationFactor(double inflationFactor) {
		if (1d <= inflationFactor) {
			this.inflationFactor = inflationFactor;
		} else {
			throw new IllegalArgumentException("inflation factor is less than 1");
		}
	}

	@Override
	public double getDeflationAmount() {
		return this.deflationAmount;
	}

	@Override
	public void setDeflationAmount(double deflationAmount) {
		if (0d < deflationAmount) {
			this.deflationAmount = deflationAmount;
		} else {
			throw new IllegalArgumentException("deflation amount is less than or equal 0");
		}
	}
	
	protected void deflate() {
		if (1d > (this.inflationFactor - this.deflationAmount)) {
			this.inflationFactor -= this.deflationAmount;
		} else {
			this.inflationFactor = 1d;
		}
	}
	
	protected boolean isDeflated() {
		return (1d == this.inflationFactor);
	}

	@Override
	public void addPlanChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("plan", listener);
	}
	
}
