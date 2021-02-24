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
	// repairCost(Waypoint)
	// setSignficantCostChange
	// isSignificantCostChange
	// notifyCostChange (implements Cost/Obstacle ChangeListener, registers at DynamicEnvironment)
	// Environment <- StaticEnvironment, DynamicEnvironment ?
	// DynamicEnvironment.registerDynamicCostListener(DynamicCostListener dcl)
}
