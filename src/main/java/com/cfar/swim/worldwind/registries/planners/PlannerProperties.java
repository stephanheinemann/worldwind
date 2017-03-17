package com.cfar.swim.worldwind.registries.planners;

import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.Properties;

/**
 * Describes planner properties applicable to all planners.
 * 
 * @author Stephan Heinemann
 *
 */
public interface PlannerProperties extends Properties<Planner> {

	/**
	 * Gets the cost policy of this planner properties bean.
	 * 
	 * @return the cost policy of this planner properties bean
	 */
	public CostPolicy getCostPolicy();
	
	/**
	 * Sets the cost policy of this planner properties bean.
	 * 
	 * @param costPolicy the cost policy to be set
	 */
	public void setCostPolicy(CostPolicy costPolicy);
	
	/**
	 * Gets the risk policy of this planner properties bean.
	 * 
	 * @return the risk policy of this planner properties bean
	 */
	public RiskPolicy getRiskPolicy();
	
	/**
	 * Sets the risk policy of this planner properties bean.
	 * 
	 * @param riskPolicy the risk policy to be set
	 */
	public void setRiskPolicy(RiskPolicy riskPolicy);
	
	/**
	 * Clones this planner properties bean.
	 * 
	 * @return a clone of this planner properties bean
	 * 
	 * @see Properties#clone()
	 */
	@Override
	public PlannerProperties clone();
	
}
