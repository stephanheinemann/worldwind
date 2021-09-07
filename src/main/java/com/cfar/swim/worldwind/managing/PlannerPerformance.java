package com.cfar.swim.worldwind.managing;

/**
 * Realizes a planner performance consisting of trajectory quality and
 * computational duration quantity.
 * 
 * @author Stephan Heinemann
 *
 * @see AbstractPerformance
 */
public class PlannerPerformance extends AbstractPerformance {
	
	/**
	 * Construct a new planner performance based on a trajectory quality and
	 * a computational duration quantity.
	 * 
	 * @param quality the trajectory quality
	 * @param quantity the computational duration quantity
	 * 
	 * @see AbstractPerformance#AbstractPerformance(Quality, Quantity)
	 */
	public PlannerPerformance(
			TrajectoryQuality quality, DurationQuantity quantity) {
		super(quality, quantity);
	}

}
