package com.cfar.swim.worldwind.ai.arastar;

import com.cfar.swim.worldwind.ai.astar.AStarWaypoint;

import gov.nasa.worldwind.geom.Position;

public class ARAStarWaypoint extends AStarWaypoint {
	
	/** the estimated previous expansion cost of this waypoint */
	protected double v = Double.POSITIVE_INFINITY;
	
	/** the weight of the estimated remaining cost of this waypoint */
	protected double epsilon = 1d;
	
	public ARAStarWaypoint(Position position) {
		super(position);
	}
	
	@Override
	public ARAStarWaypoint getParent() {
		return (ARAStarWaypoint) super.getParent();
	}
	
	/**
	 * Gets the estimated previous expansion cost of this waypoint.
	 * 
	 * @return the estimated previous expansion cost of this waypoint
	 */
	public double getV() {
		return this.v;
	}
	
	/**
	 * Sets the estimated previous expansion cost of this waypoint.
	 * 
	 * @param v the estimated previous expansion cost of this waypoint
	 */
	public void setV(double v) {
		this.v = v;
	}
	
	/**
	 * Gets the weight of the estimated remaining cost of this waypoint.
	 * 
	 * @return the weight of the estimated remaining cost of this waypoint
	 */
	public double getEpsilon() {
		return this.epsilon;
	}
	
	/**
	 * Sets the weight of the estimated remaining cost of this waypoint.
	 * 
	 * @param epsilon the weight of the estimated remaining cost of this
	 *                waypoint
	 */
	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}
	
	/**
	 * Gets the estimated total cost of this waypoint.
	 * 
	 * @return the estimated total cost of this waypoint
	 */
	@Override
	public double getF() {
		return this.g + (this.epsilon * this.h);
	}
	
	// TODO: override compareTo ?

}
