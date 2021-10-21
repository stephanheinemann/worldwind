/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.cfar.swim.worldwind.registries.planners.rrt;

import java.util.Objects;

import javax.validation.constraints.DecimalMax;
import javax.validation.constraints.DecimalMin;
import javax.validation.constraints.Max;
import javax.validation.constraints.Min;

import com.cfar.swim.worldwind.planners.rrt.Extension;
import com.cfar.swim.worldwind.planners.rrt.Sampling;
import com.cfar.swim.worldwind.planners.rrt.Strategy;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.planners.AbstractPlannerProperties;

/**
 * Realizes the properties bean of a basic RRT planner.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class RRTreeProperties extends AbstractPlannerProperties {
	
	/** the default serial identification of this RRT planner properties bean */
	private static final long serialVersionUID = 1L;
	
	/** the sampling distribution of this RRT planner properties bean */
	private Sampling sampling = Sampling.UNIFORM;
	
	/** the expansion strategy of this RRT planner properties bean */
	private Strategy strategy = Strategy.EXTEND;
	
	/** the extension technique of this RRT planner properties bean */
	private Extension extension = Extension.LINEAR;
	
	/** the maximum number of sampling iterations of this RRT planner properties bean */
	@Min(value = 1, message = "{property.planner.rrt.maxIterations.min}")
	@Max(value = Integer.MAX_VALUE, message = "{property.planner.rrt.maxIterations.max}")
	private int maxIterations = 3_000;
	
	/** the maximum extension distance to a waypoint in the tree of this RRT planner properties bean */
	@DecimalMin(value = "1", message = "{property.planner.rrt.epsilon.min}")
	@DecimalMax(value = "100000", message = "{property.planner.rrt.epsilon.max}")
	private double epsilon = 25d;
	
	/** the sampling bias towards the goal of this RRT planner properties bean */
	@Min(value = 1, message = "{property.planner.rrt.bias.min}")
	@Max(value = 100, message = "{property.planner.rrt.bias.max}")
	private int bias = 5;
	
	/** the radius of the sphere defining the goal region of this RRT planner properties bean */
	@DecimalMin(value = "0", message = "{property.planner.rrt.goalThreshold.min}")
	@DecimalMax(value = "100000", message = "{property.planner.rrt.goalThreshold.max}")
	private double goalThreshold = 1d;
	
	/**
	 * Constructs a new basic RRT planner properties bean.
	 */
	public RRTreeProperties() {
		super();
	}
	
	/**
	 * Constructs a new basic RRT planner properties bean with specified cost
	 * and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this basic RRT planner properties
	 *                   bean
	 * @param riskPolicy the risk policy of this basic RRT planner properties
	 *                   bean
	 */
	public RRTreeProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
	}
	
	/**
	 * Constructs a new basic RRT planner properties bean with specified cost
	 * and risk policy property values as well as specified sampling
	 * distribution, expansion strategy, extension technique, maximum number of
	 * iterations, maximum distance for extensions, and goal bias.
	 * 
	 * @param costPolicy    the cost policy of this basic RRT planner
	 *                      properties bean
	 * @param riskPolicy    the risk policy of this basic RRT planner
	 *                      properties bean
	 * @param sampling      the sampling distribution of this basic RRT planner
	 *                      properties bean
	 * @param strategy      the expansion strategy of this basic RRT planner
	 *                      properties bean
	 * @param extension		the extension technique of this basic RRT planner
	 *                      properties bean
	 * @param maxIterations the maximum number of sampling iterations of this
	 *                      basic RRT planner properties bean
	 * @param epsilon       the maximum extension distance to a waypoint in the
	 *                      tree of this basic RRT planner properties bean
	 * @param bias          the sampling bias towards the goal of this basic
	 *                      RRT planner properties bean
	 * @param goalThreshold the radius of the sphere defining the goal region
	 *                      of this basic RRT planner properties bean 
	 */
	public RRTreeProperties(CostPolicy costPolicy, RiskPolicy riskPolicy,
			Sampling sampling, Strategy strategy, Extension extension,
			int maxIterations, double epsilon, int bias, double goalThreshold) {
		super(costPolicy, riskPolicy);
		this.setSampling(sampling);
		this.setStrategy(strategy);
		this.setExtension(extension);
		this.setMaxIterations(maxIterations);
		this.setEpsilon(epsilon);
		this.setBias(bias);
		this.setGoalThreshold(goalThreshold);
	}
	
	/**
	 * Gets the sampling distribution of this RRT planner properties bean.
	 * 
	 * @return the sampling distribution of this RRT planner properties bean
	 */
	public Sampling getSampling() {
		return this.sampling;
	}
	
	/**
	 * Sets the sampling distribution of this RRT planner properties bean.
	 * 
	 * @param sampling the sampling distribution to be set
	 */
	public void setSampling(Sampling sampling) {
		this.sampling = sampling;
	}
	
	/**
	 * Gets the expansion strategy of this RRT planner properties bean.
	 * 
	 * @return the expansion strategy of this RRT planner properties bean
	 */
	public Strategy getStrategy() {
		return this.strategy;
	}
	
	/**
	 * Sets the expansion strategy of this RRT planner properties bean
	 * 
	 * @param strategy the expansion strategy to be set
	 */
	public void setStrategy(Strategy strategy) {
		this.strategy = strategy;
	}
	
	/**
	 * Gets the extension technique of this RRT planner properties bean.
	 * 
	 * @return the extension technique of this RRT planner properties bean
	 */
	public Extension getExtension() {
		return this.extension;
	}
	
	/**
	 * Sets the extension technique of this RRT planner properties bean.
	 * 
	 * @param extension the extension technique to be set
	 */
	public void setExtension(Extension extension) {
		this.extension = extension;
	}
	
	/**
	 * Gets the maximum number of sampling iterations towards the goal of this
	 * RRT planner properties bean.
	 * 
	 * @return the maximum number of sampling iterations of this RRT planner
	 *         properties bean
	 */
	public int getMaxIterations() {
		return this.maxIterations;
	}
	
	/**
	 * Sets the maximum number of sampling iterations towards the goal of this
	 * RRT planner properties bean.
	 * 
	 * @param maxIterations the maximum number of sampling iterations to be set
	 */
	public void setMaxIterations(int maxIterations) {
		this.maxIterations = maxIterations;
	}
	
	/**
	 * Gets the maximum extension distance to a waypoint in the tree of this
	 * RRT planner properties bean.
	 * 
	 * @return the maximum extension distance of this RRT planner properties
	 *         bean
	 */
	public double getEpsilon() {
		return this.epsilon;
	}
	
	/**
	 * Sets the maximum extension distance to a waypoint in the tree of this
	 * RRT planner properties bean.
	 * 
	 * @param epsilon the maximum extension distance to be set
	 */
	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}
	
	/**
	 * Gets the sampling bias towards the goal of this RRT planner properties
	 * bean.
	 * 
	 * @return the sampling bias towards the goal of this RRT planner
	 *         properties bean
	 */
	public int getBias() {
		return this.bias;
	}
	
	/**
	 * Sets the sampling bias towards the goal of this RRT planner properties
	 * bean.
	 * 
	 * @param bias the sampling bias to be set
	 */
	public void setBias(int bias) {
		this.bias = bias;
	}
	
	/**
	 * Gets the the radius of the sphere defining the goal region of this RRT
	 * planner properties bean.
	 * 
	 * @return the radius of the sphere defining the goal region of this RRT
	 *         planner properties bean
	 */
	public double getGoalThreshold() {
		return this.goalThreshold;
	}
	
	/**
	 * Sets the radius of the sphere defining the goal region of this RRT
	 * planner properties bean.
	 * 
	 * @param goalThreshold the radius of the sphere defining the goal region
	 *                      to be set
	 */
	public void setGoalThreshold(double goalThreshold) {
		this.goalThreshold = goalThreshold;
	}
	
	/**
	 * Determines whether or not this RRT planner properties bean equals
	 * another RRT planner properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other RRT planner properties bean
	 * 
	 * @return true, if the aggregated properties of this RRT planner
	 *         properties bean equal the aggregated properties of the other
	 *         RRT planner properties bean, false otherwise
	 * 
	 * @see AbstractPlannerProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			RRTreeProperties rrtp = (RRTreeProperties) o;
			equals = (this.bias == rrtp.bias)
					&& (this.epsilon == rrtp.epsilon)
					&& (this.extension == rrtp.extension)
					&& (this.goalThreshold == rrtp.goalThreshold)
					&& (this.maxIterations == rrtp.maxIterations)
					&& (this.sampling == rrtp.sampling)
					&& (this.strategy == rrtp.strategy);
		} else {
			equals = false;
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this RRT planner properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this RRT planner properties bean based on its
	 *         aggregated properties
	 * 
	 * @see AbstractPlannerProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.bias,
				this.epsilon,
				this.extension,
				this.goalThreshold,
				this.maxIterations,
				this.sampling,
				this.strategy);
	}
	
}
