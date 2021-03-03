package com.cfar.swim.worldwind.ai;

/**
 * Describes a dynamic planner which repairs changing costs while planning.
 * A dynamic planner only revises invalid parts of the plan being computed
 * and avoids the revision of the entire plan as much as possible.
 * 
 * @author Stephan Heinemann
 * 
 */
public interface DynamicPlanner extends Planner, DynamicCostListener {

	// TODO: merge with and check for PRM/RTT interface
	
	/**
	 * Terminates this dynamic planner.
	 */
	public void terminate();
	
	/**
	 * Indicates whether or not this dynamic planner has terminated.
	 * 
	 * @return true if this dynamic planner has terminated, false otherwise
	 */
	public boolean hasTerminated();
	
	/**
	 * Sets the significant change threshold of this dynamic planner.
	 * 
	 * @param significantChange the signficant change threshold of this dynamic
	 *                          planner
	 */
	public void setSignificantChange(double significantChange);
	
	/**
	 * Indicates whether or or not this dynamic planner has a significant
	 * dynamic change.
	 * 
	 * @return true if this dynamic planner has a significant dynamic change,
	 *         false otherwise
	 */
	public boolean hasSignificantChange();
	
	// repairCost(Waypoint)
	
	// significant cost = affected legs / current legs (e.g. accept 0.25)
	// changes close to start versus goal (resulting repair)
	
	// setSignficantCostChange
	// isSignificantCostChange
	// notifyCostChange (implements Cost/Obstacle ChangeListener, registers at DynamicEnvironment)
	// Environment <- StaticEnvironment, DynamicEnvironment ?
	// DynamicEnvironment.registerDynamicCostListener(DynamicCostListener dcl)
}
