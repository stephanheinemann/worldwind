package com.cfar.swim.worldwind.ai.astar;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

public class AStarWaypoint extends Waypoint {
	
	/** the estimated current cost of this waypoint */
	protected double g = Double.POSITIVE_INFINITY;
	
	/** the estimated remaining cost of this waypoint */
	protected double h = Double.POSITIVE_INFINITY;
	
	public AStarWaypoint(Position position) {
		super(position);
	}
	
	@Override
	public AStarWaypoint getParent() {
		return (AStarWaypoint) super.getParent();
	}
	
	/**
	 * Gets the estimated current cost of this waypoint.
	 * 
	 * @return the estimated current cost of this waypoint
	 */
	public double getG() {
		return this.g;
	}
	
	/**
	 * Sets the estimated current cost of this waypoint.
	 * 
	 * @param g the estimated current cost of this waypoint
	 */
	public void setG(double g) {
		this.g = g;
	}
	
	/**
	 * Gets the estimated remaining cost of this waypoint.
	 * 
	 * @return the estimated remaining cost of this waypoint
	 */
	public double getH() {
		return this.h;
	}
	
	/**
	 * Sets the estimated remaining cost of this waypoint.
	 * 
	 * @param h the estimated remaining cost of this waypoint
	 */
	public void setH(double h) {
		this.h = h;
	}
	
	/**
	 * Gets the estimated total cost of this waypoint.
	 * 
	 * @return the estimated total cost of this waypoint
	 */
	public double getF() {
		return this.g + this.h;
	}
	
	/**
	 * Compares this waypoint to another waypoint based on their estimated
	 * total costs. It the estimated total costs of both waypoints is equal,
	 * then break ties in favor of higher estimated current costs.
	 * 
	 * @param o the other waypoint
	 * 
	 * @return -1, 0, 1, if this waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their total
	 *         estimated cost
	 * 
	 * @see Comparable#compareTo(Object)
	 */
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;
		
		if (waypoint instanceof AStarWaypoint) {
			compareTo = new Double(this.getF())
					.compareTo(((AStarWaypoint) waypoint).getF());
			if (0 == compareTo) {
				// break ties in favor of higher G-values
				compareTo = new Double(((AStarWaypoint) waypoint).getG())
						.compareTo(this.getG());
			}
		} else {
			compareTo = super.compareTo(waypoint);
		}
		
		return compareTo;
	}

}
