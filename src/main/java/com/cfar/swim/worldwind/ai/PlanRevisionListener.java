package com.cfar.swim.worldwind.ai;

import java.util.EventListener;

import com.cfar.swim.worldwind.planning.Trajectory;

/**
 * Describes a plan revision listener that will be invoked by its planner
 * whenever a plan has been revised.
 * 
 * @author Stephan Heinemann
 * 
 */
public interface PlanRevisionListener extends EventListener {

	/**
	 * Notifies this plan revision listener about a revised plan.
	 * 
	 * @param trajectory the revised trajectory
	 */
	public void revisePlan(Trajectory trajectory);
	
}
