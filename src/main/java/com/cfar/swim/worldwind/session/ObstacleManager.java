package com.cfar.swim.worldwind.session;

import java.util.Set;

import com.cfar.swim.worldwind.render.Obstacle;

/**
 * Describes an obstacle manager responsible for requesting and committing
 * obstacle changes.
 * 
 * @author Stephan Heinemann
 *
 */
public interface ObstacleManager {

	/**
	 * Submits an obstacle change (addition or removal) to this obstacle
	 * manager.
	 * 
	 * @param obstacles the obstacles to be changed
	 */
	public void submitObstacleChange(Set<Obstacle> obstacles);
	
	/**
	 * Commits an obstacle change (addition or removal) to this obstacle
	 * manager.
	 * 
	 * @return the obstacles that were changed
	 */
	public Set<Obstacle> commitObstacleChange();
	
	/**
	 * Retracts an obstacle change (addition or removal) from this obstacle
	 * manager.
	 */
	public void retractObstacleChange();
	
	/**
	 * Determines whether or not this obstacle manager has an obstacle change.
	 * 
	 * @return true if this obstacle manager has an obstacle change,
	 *         false otherwise
	 */
	public boolean hasObstacleChange();
	
}
