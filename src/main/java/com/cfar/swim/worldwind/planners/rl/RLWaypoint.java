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
	private Set<Position> neighbors = new LinkedHashSet<Position>();

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
	public void setNeighbors(Set<Position> neighbors) {
		Iterator<Position> neighborsIterator = neighbors.iterator();
		while (neighborsIterator.hasNext()) {
			Position p = neighborsIterator.next();
			this.neighbors.add(p);
		}
	}
	
	/**
	 * Gets the neighbor correspondent to the chosen action index.
	 * 
	 * @param chosen action index
	 * 
	 * @return neighbor posiition
	 */
	public Position getNeighbor(int index) {
		Iterator<Position> neighborsIterator = neighbors.iterator();
		int i = 0;
		while (neighborsIterator.hasNext() || i < index) {
			i++;
		}
		return neighborsIterator.next();
	}
}
