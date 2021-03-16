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
	 * Request an obstacle change (addition or removal).
	 * 
	 * @param obstacles the obstacles to be changed
	 */
	public void requestObstacleChange(Set<Obstacle> obstacles);
	
	/**
	 * Commits an obstacle change (addition or removal).
	 * 
	 * @return the obstacles that were changed
	 */
	public Set<Obstacle> commitObstacleChange();
	
	/**
	 * Retracts an obstacle change (addition or removal).
	 */
	public void retractObstacleChange();
	
}
