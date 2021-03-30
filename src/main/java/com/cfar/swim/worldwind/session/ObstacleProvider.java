package com.cfar.swim.worldwind.session;

/**
 * Describes an obstacle provider.
 * 
 * @author Stephan Heinemann
 *
 */
public interface ObstacleProvider {

	/**
	 * Gets the obstacle manager of this obstacle provider.
	 * 
	 * @return the obstacle manager of this obstacle provider
	 */
	public ObstacleManager getObstacleManager();
	
	/**
	 * Sets the obstacle manager of this obstacle provider.
	 * 
	 * @param obstacleManager the obstacle manager to be set
	 */
	public void setObstacleManager(ObstacleManager obstacleManager);
	
	/**
	 * Determines whether or not this obstacle provider has an obstacle manager.
	 * 
	 * @return true if this obstacle provider has an obstacle manager,
	 *         false otherwise
	 */
	public boolean hasObstacleManager();
	
	/**
	 * Gets whether or not this obstacle provider automatically commits
	 * obstacle changes.
	 * 
	 * @return true if this obstacle provider automatically commits obstacle
	 *         changes, false otherwise
	 */
	public boolean getAutoCommit();
	
	/**
	 * Sets whether or not this obstacle provider automatically commits
	 * obstacle changes.
	 * 
	 * @param autoCommit true if this obstacle provider automatically commits
	 *                   obstacle changes, false otherwise
	 */
	public void setAutoCommit(boolean autoCommit);
	
}
