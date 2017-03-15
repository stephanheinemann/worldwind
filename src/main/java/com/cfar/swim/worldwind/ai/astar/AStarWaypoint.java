package com.cfar.swim.worldwind.ai.astar;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an A* waypoint of a trajectory featuring estimates for costs and
 * time. A* waypoints are used by A* planners. 
 * 
 * @author Stephan Heinemann
 *
 */
public class AStarWaypoint extends Waypoint {
	
	/** the parent A* waypoint of this A* waypoint in a trajectory */
	private AStarWaypoint parent = null;
	
	/** the estimated remaining cost (h-value) of this A* waypoint */
	private double h;
	
	/**
	 * Constructs an A* waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see Waypoint#Waypoint(Position)
	 */
	public AStarWaypoint(Position position) {
		super(position);
		this.setG(Double.POSITIVE_INFINITY);
		this.setH(Double.POSITIVE_INFINITY);
	}
	
	/**
	 * Gets the parent A* waypoint of this A* waypoint.
	 * 
	 * @return the parent A* waypoint of this A* waypoint
	 */
	public AStarWaypoint getParent() {
		return parent;
	}
	
	/**
	 * Sets the parent A* waypoint of this A* waypoint.
	 * 
	 * @param parent the parent A* waypoint of this A* waypoint
	 */
	public void setParent(AStarWaypoint parent) {
		this.parent = parent;
	}
	
	/**
	 * Gets the estimated current cost (g-value) of this A* waypoint.
	 * 
	 * @return the estimated current cost (g-value) of this A* waypoint
	 */
	public double getG() {
		return super.getCost();
	}
	
	/**
	 * Sets the estimated current cost (g-value) of this A* waypoint.
	 * 
	 * @param g the estimated current cost (g-value) of this A* waypoint
	 */
	public void setG(double g) {
		if (0d > g) {
			throw new IllegalArgumentException("g is less than 0");
		}
		super.setCost(g);
	}
	
	/**
	 * Gets the estimated remaining cost (h-value) of this A* waypoint.
	 * 
	 * @return the estimated remaining cost (h-value) of this A* waypoint
	 */
	public double getH() {
		return this.h;
	}
	
	/**
	 * Sets the estimated remaining cost (h-value) of this A* waypoint.
	 * 
	 * @param h the estimated remaining cost (h-value) of this A* waypoint
	 */
	public void setH(double h) {
		if (0d > h) {
			throw new IllegalArgumentException("h is less than 0");
		}
		this.h = h;
	}
	
	/**
	 * Gets the estimated total cost (f-value) of this A* waypoint.
	 * 
	 * @return the estimated total cost (f-value) of this A* waypoint
	 */
	public double getF() {
		return this.getG() + this.getH();
	}
	
	/**
	 * Clones this A* waypoint without its parent and depiction.
	 * 
	 * @return the clone of this A* waypoint without its parent and depiction
	 * 
	 * @see Object#clone()
	 */
	@Override
	public AStarWaypoint clone() {
		AStarWaypoint waypoint = (AStarWaypoint) super.clone();
		waypoint.setParent(null);
		return waypoint;
	}
	
	/**
	 * Compares this A* waypoint to another waypoint based on their estimated
	 * total costs (f-values). It the estimated total costs of both A*
	 * waypoints is equal, then ties are broken in favor of higher estimated
	 * current costs (g-values). If the other waypoint is not an A* waypoint,
	 * then the natural order of waypoint applies.
	 * 
	 * @param waypoint the other waypoint
	 * 
	 * @return -1, 0, 1, if this A* waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their total
	 *         estimated cost
	 * 
	 * @see Waypoint#compareTo(Waypoint)
	 */
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;
		
		if (waypoint instanceof AStarWaypoint) {
			AStarWaypoint asWaypoint = (AStarWaypoint) waypoint;
			compareTo = new Double(this.getF()).compareTo(asWaypoint.getF());
			if (0 == compareTo) {
				// break ties in favor of higher G-values
				compareTo = new Double(asWaypoint.getG()).compareTo(this.getG());
			}
		} else {
			compareTo = super.compareTo(waypoint);
		}
		
		return compareTo;
	}

}
