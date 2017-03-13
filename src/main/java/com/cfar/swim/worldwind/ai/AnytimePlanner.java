package com.cfar.swim.worldwind.ai;

/**
 * Describes an anytime planner which issues plans featuring a specified
 * minimum quality or better. The anytime planner continuously attempts to
 * revise a previously issued plan by incrementally improving its quality
 * until a maximum quality or better has been achieved.
 * 
 * @author Stephan Heinemann
 * 
 */
public interface AnytimePlanner extends Planner {
	
	/**
	 * Gets the minimum required quality of a plan issued by this anytime
	 * planner.
	 * 
	 * @return the minimum required quality of a plan issued by this anytime
	 *         planner
	 */
	public double getMinimumQuality();
	
	/**
	 * Sets the minimum required quality of a plan issued by this anytime
	 * planner.
	 * 
	 * @param minQuality the minimum required quality of a plan issued by this
	 *                   anytime planner
	 */
	public void setMinimumQuality(double minQuality);
	
	/**
	 * Gets the maximum required quality of a plan issued by this anytime
	 * planner.
	 * 
	 * @return the maximum required quality of a plan issued by this anytime
	 *         planner
	 */
	public double getMaximumQuality();
	
	/**
	 * Sets the maximum required quality of a plan issued by this anytime
	 * planner.
	 * 
	 * @param maxQuality the maximum required quality of a plan issued by this
	 *                   anytime planner
	 */
	public void setMaximumQuality(double maxQuality);
	
	/**
	 * Gets the required quality improvement between consecutively issued plans
	 * employed by this anytime planner.
	 * 
	 * @return the required quality improvement employed by this anytime
	 *         planner
	 */
	public double getQualityImprovement();
	
	/**
	 * Sets the required quality improvement between consecutively issued plans
	 * employed by this anytime planner.
	 * 
	 * @param improvement the required quality improvement employed by this
	 *                    anytime planner
	 */
	public void setQualityImprovement(double improvement);
	
}
