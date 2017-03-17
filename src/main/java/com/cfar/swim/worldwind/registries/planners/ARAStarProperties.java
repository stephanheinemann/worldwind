package com.cfar.swim.worldwind.registries.planners;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;

/**
 * Realizes the properties bean of an ARA* planner.
 * 
 * @author Stephan Heinemann
 *
 */
public class ARAStarProperties extends ForwardAStarProperties implements AnytimePlannerProperties {
	
	/** the initial inflation factor applied to the heuristic function */
	private double initialInflation;
	
	/** the final inflation factor applied the heuristic function */
	private double finalInflation;
	
	/** the deflation amount to be applied to the current inflation */
	private double deflationAmount;
	
	/**
	 * Constructs a new ARA* planner properties bean.
	 */
	public ARAStarProperties() {
		super();
		this.setMinimumQuality(1d);
		this.setMaximumQuality(1d);
		this.setQualityImprovement(1d);
	}
	
	/**
	 * Constructs a new ARA* planner properties bean with
	 * specified cost and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this ARA* planner properties bean
	 * @param riskPolicy the risk policy of this ARA* planner properties bean
	 */
	public ARAStarProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
		this.setMinimumQuality(1d);
		this.setMaximumQuality(1d);
		this.setQualityImprovement(1d);
	}
	
	/**
	 * Constructs a new ARA* planner properties bean with specified cost and
	 * risk policy property values as well as specified initial, final and
	 * decrease of heuristic inflation property values.
	 * 
	 * @param costPolicy the cost policy of this ARA* planner properties bean
	 * @param riskPolicy the risk policy of this ARA* planner properties bean
	 * @param initialInflation the initial inflation of the heuristic function
	 * @param finalInflation the final inflation of the heuristic function
	 * @param deflationAmount the deflation amount of the heuristic function
	 */
	public ARAStarProperties(
			CostPolicy costPolicy, RiskPolicy riskPolicy,
			double initialInflation,
			double finalInflation,
			double deflationAmount) {
		super(costPolicy, riskPolicy);
		this.setMinimumQuality(initialInflation);
		this.setMaximumQuality(finalInflation);
		this.setQualityImprovement(deflationAmount);
	}
	
	/**
	 * Gets the minimum quality (initial inflation) of this
	 * ARA* properties bean.
	 * 
	 * @return the minimum quality (initial inflation) of this
	 *         ARA* properties bean
	 * 
	 * @see AnytimePlannerProperties#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialInflation;
	}
	
	/**
	 * Sets the minimum quality (initial inflation) of this
	 * ARA* properties bean.
	 * 
	 * @param initialInflation the minimum quality (initial inflation) of this
	 *                         ARA* properties bean
	 * 
	 * @throws IllegalArgumentException if the initial inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialInflation) {
		if ((1d <= initialInflation) &&
				(initialInflation >= this.finalInflation)) {
			this.initialInflation = initialInflation;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}
	
	/**
	 * Gets the maximum quality (final inflation) of this ARA* properties bean.
	 * 
	 * @return the maximum quality (final inflation) of this
	 *         ARA* properties bean
	 * 
	 * @see AnytimePlannerProperties#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalInflation;
	}
	
	/**
	 * Sets the maximum quality (initial inflation) of this
	 * ARA* properties bean.
	 * 
	 * @param finalInflation the maximum quality (final inflation) of this
	 *                       ARA* properties bean
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalInflation) {
		if ((1d <= finalInflation) && (this.initialInflation >= finalInflation)) {
			this.finalInflation = finalInflation;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}
	
	/**
	 * Gets the quality improvement (deflation amount) of this
	 * ARA* properties bean.
	 * 
	 * @return the quality improvement (deflation amount) of this
	 *         ARA* properties bean
	 * 
	 * @see AnytimePlannerProperties#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.deflationAmount;
	}

	/**
	 * Sets the quality improvement (deflation amount) of this
	 * ARA* properties bean.
	 * 
	 * @param deflationAmount the quality improvement (deflation amount) of
	 *                        this ARA* properties bean
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlannerProperties#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double deflationAmount) {
		if (0d < deflationAmount) {
			this.deflationAmount = deflationAmount;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}

}
