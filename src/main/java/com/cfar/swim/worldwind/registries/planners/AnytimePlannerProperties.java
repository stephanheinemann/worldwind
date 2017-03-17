package com.cfar.swim.worldwind.registries.planners;

/**
 * Describes the properties bean of an anytime planner.
 * 
 * @author Stephan Heinemann
 *
 */
public interface AnytimePlannerProperties extends PlannerProperties {
	
	/**
	 * Gets the minimum required quality of a plan of this anytime planner
	 * properties bean.
	 * 
	 * @return the minimum required quality of a plan of this anytime planner
	 *         properties bean
	 */
	public double getMinimumQuality();
	
	/**
	 * Sets the minimum required quality of a plan of this anytime planner
	 * properties bean.
	 * 
	 * @param minQuality the minimum required quality of a plan of this anytime
	 *                   planner properties bean
	 */
	public void setMinimumQuality(double minQuality);
	
	/**
	 * Gets the maximum required quality of a plan of this anytime planner
	 * properties bean.
	 * 
	 * @return the maximum required quality of a plan of this anytime planner
	 *         properties bean
	 */
	public double getMaximumQuality();
	
	/**
	 * Sets the maximum required quality of a plan of this anytime planner
	 * properties bean.
	 * 
	 * @param maxQuality the maximum required quality of a plan of this anytime
	 *                   planner properties bean
	 */
	public void setMaximumQuality(double maxQuality);
	
	/**
	 * Gets the required quality improvement between consecutively issued plans
	 * of this anytime planner properties bean.
	 * 
	 * @return the required quality improvement of this anytime planner
	 *         properties bean
	 */
	public double getQualityImprovement();
	
	/**
	 * Sets the required quality improvement between consecutively issued plans
	 * of this anytime planner properties bean.
	 * 
	 * @param improvement the required quality improvement of this anytime
	 *                    planner properties bean
	 */
	public void setQualityImprovement(double improvement);

}
