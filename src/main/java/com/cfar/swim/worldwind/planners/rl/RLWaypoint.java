package com.cfar.swim.worldwind.planners.rl;

import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a RL waypoint of a trajectory planned by RL planners.
 * 
 * @author Rafaela Seguro
 *
 */
public class RLWaypoint extends Waypoint {
	
	/** the children RRT waypoints of this RRT waypoint in a tree */
	private Set<RLWaypoint> neighbors = new LinkedHashSet<RLWaypoint>();
	
	/** int to ID the waypoint */
	private int id = 0;

	/** the cost associated to this waypoint */
	private double cost = 0;


	/**
	 * Constructs a RL waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see Waypoint#Waypoint(Position)
	 */
	public RLWaypoint(Position position) {
		super(position);
	}
	
	/**
	 * Sets the RL waypoint's neighbor set.
	 * 
	 * @param set of waypoint's neighbors
	 */
	public void setNeighbors(Set<RLWaypoint> neighbors) {
		Iterator<RLWaypoint> neighborsIterator = neighbors.iterator();
		while (neighborsIterator.hasNext()) {
			RLWaypoint p = neighborsIterator.next();
			this.neighbors.add(p);
		}
	}
	
	/**
	 * Gets the RL waypoint's neighbor set.
	 * 
	 * @param set of waypoint's neighbors
	 */
	public Set<RLWaypoint>  getNeighbors() {
		return neighbors;
	}
	
	/**
	 * Sets the RL waypoint's cost.
	 * 
	 * @param cost waypoint's cost
	 */
	public void  setCost(double cost) {
		this.cost = cost;
	}
	
	/**
	 * Gets the RL waypoint's cost.
	 * 
	 * @return waypoint's cost
	 */
	public double getCost() {
		return this.cost;
	}
	
	/**
	 * Gets the neighbor correspondent to the chosen action index.
	 * 
	 * @param chosen action index
	 * 
	 * @return neighbor position
	 */
	public RLWaypoint getNeighbor(int index) {
		Iterator<RLWaypoint> neighborsIterator = neighbors.iterator();
		int i = 0;
		while (neighborsIterator.hasNext() && i < index) {
			neighborsIterator.next();
			i++;
		}
		return neighborsIterator.next();
	}
	
	/**
	 * Sets the RL waypoint's ID.
	 * 
	 * @param id waypoint's ID
	 */
	public void setId(int id) {
		this.id = id;
	}
	
	
	/**
	 * Gets the RL waypoint's ID.
	 * 
	 * @return id waypoint's ID
	 */
	public int getId() {
		return this.id;
	}
	
	
	/**
	 * Compares this RL waypoint to another waypoint based on their position.
	 * Note that this implementation is *not* consistent with equals and,
	 * therefore, sorted sets cannot be used with waypoints.
	 * 
	 * @param RL waypoint the other RL waypoint
	 * 
	 * @return -1, 0, 1, if this waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their position
	 * 
	 * @see Comparable#compareTo(Object)
	 */
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;
		
		compareTo = this.getPrecisionPosition().compareTo(waypoint.getPrecisionPosition());
		
		return compareTo;
	}
	
	/**
	 * Gets the distance between this RL Waypoint's coordinates and another's in km.
	 * 
	 * @param RL waypoint the other RL waypoint
	 * 
	 * @return the distance in km
	 * 
	 * @see Comparable#compareTo(Object)
	 */
	public double getDistance(Waypoint waypoint) {
		double p = Math.PI/180;
		double a = 0.5 - Math.cos((this.getLatitude().radians - waypoint.getLatitude().radians)*p)/2 +
				Math.cos(waypoint.getLatitude().radians * p) * Math.cos(this.getLatitude().radians * p) *
				(1 - Math.cos((this.getLongitude().radians - waypoint.getLongitude().radians)*p))/2;
		
		return 12742 * Math.asin(Math.sqrt(a));
	}
}
