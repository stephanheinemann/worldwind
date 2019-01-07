/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.registries.planners;

import com.cfar.swim.worldwind.ai.prm.rigidprm.CollisionDelay;
import com.cfar.swim.worldwind.ai.prm.rigidprm.EnhancementMode;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryMode;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryPlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Sampling;

/**
 * Realizes the properties bean of a rigid PRM planner.
 * 
 * @author Henrique Ferreira
 *
 */
public class RigidPRMProperties extends AbstractPlannerProperties implements AnytimePlannerProperties {

	/** the description of this planner properties bean */
	private final static String DESCRIPTION_RIGIDPRM = "Probabilistic Roadmap: Constructs a roadmap by sampling points in a continuous environment. Then, a deterministic planner is used to compute the desired trajectory.";

	/** the maximum number of sampling iterations in the construction step */
	protected int maxIterConstruction = 500;

	/** the maximum number of sampling iterations in the enhancement step */
	protected int maxIterEnhancement = 0;

	/** the permission to connect RigidPRMWaypoints in the same connected component */
	protected boolean sameComponent = true;

	/** the maximum number of neighbors a RigidPRMWaypoint can be connected to */
	protected int maxNeighbors = 15;

	/** the maximum distance between two neighboring RigidPRMWaypoints */
	protected double maxDistance = 30;

	/** the selection of the variable maximum neighbors expression of PRM* */
	protected boolean optimalMaxNeighbors = false;

	/** the selection of the variable maximum distance expression of PRM* */
	protected boolean optimalMaxDistance = false;

	/** the sampling strategy to be applied in the construction step */
	protected Sampling samplingStrategy = Sampling.UNIFORM;

	/** the enhancement technique employed in the enhancement step */
	protected EnhancementMode enhancement = EnhancementMode.NONE;

	/** the type of collision delay technique for RigidPRMWaypoints and edges */
	protected CollisionDelay delayCollision = CollisionDelay.NONE;

	/** the planner used to find a path in a previously populated roadmap */
	protected QueryPlanner planner = QueryPlanner.FAS;

	/** the query mode of this PRM planner */
	protected QueryMode mode = QueryMode.SINGLE;

	/** the initial inflation factor applied to the heuristic function */
	private double minimumQuality = 50d;

	/** the final inflation factor applied to the heuristic function */
	private double maximumQuality = 1d;

	/** the inflation amount to be applied to the current inflation */
	private double qualityImprovement = 1d;
	
	/**
	 * Constructs a new rigid PRM planner properties bean.
	 */
	public RigidPRMProperties() {
		super();
		this.setDescription(DESCRIPTION_RIGIDPRM);
	}

	/**
	 * Gets the maximum number of sampling iterations in the construction step.
	 * 
	 * @return the maxIterConstruction the maximum number of sampling iterations in
	 *         the construction step
	 */
	public int getMaxIterConstruction() {
		return maxIterConstruction;
	}

	/**
	 * Sets the maximum number of sampling iterations in the construction step.
	 * 
	 * @param maxIterConstruction the maximum number of sampling iterations in the
	 *            construction step to set
	 */
	public void setMaxIterConstruction(int maxIter) {
		this.maxIterConstruction = maxIter;
	}

	/**
	 * Gets the maximum number of sampling iterations in the enhancement step.
	 * 
	 * @return the maxIterEnhancement the maximum number of sampling iterations in
	 *         the enhancement step
	 */
	public int getMaxIterEnhancement() {
		return maxIterEnhancement;
	}

	/**
	 * Sets the maximum number of sampling iterations in the enhancement step.
	 * 
	 * @param maxIterEnhancement the maximum number of sampling iterations in the
	 *            enhancement step to set
	 */
	public void setMaxIterEnhancement(int maxIterEnhancement) {
		this.maxIterEnhancement = maxIterEnhancement;
	}

	/**
	 * Checks if the permission to connect RigidPRMWaypoints in the same connected
	 * component is active.
	 * 
	 * @return true if it allows to connect RigidPRMWaypoints in the same connected
	 *         component, false otherwise
	 */
	public boolean isSameComponent() {
		return sameComponent;
	}

	/**
	 * Sets the permission to connect RigidPRMWaypoints in the same connected
	 * component.
	 * 
	 * @param sameComponent the permission level to set
	 */
	public void setSameComponent(boolean sameComponent) {
		this.sameComponent = sameComponent;
	}

	/**
	 * Gets the maximum number of neighbors a RigidPRMWaypoint can have.
	 * 
	 * @return the maxNeighbors the maximum number of neighbors a RigidPRMWaypoint
	 *         can have
	 */
	public int getMaxNeighbors() {
		return maxNeighbors;
	}

	/**
	 * Sets the maximum number of neighbors a RigidPRMWaypoint can have.
	 * 
	 * @param maxNeighbors the maximum number of neighbors a RigidPRMWaypoint can
	 *            have
	 */
	public void setMaxNeighbors(int maxNeighbors) {
		this.maxNeighbors = maxNeighbors;
	}

	/**
	 * Gets the maximum distance between two connected RigidPRMWaypoints.
	 * 
	 * @return the maxDistance the maximum distance between two connected
	 *         RigidPRMWaypoints
	 */
	public double getMaxDistance() {
		return maxDistance;
	}

	/**
	 * Sets the maximum distance between two connected RigidPRMWaypoints.
	 * 
	 * @param maxDistance the maximum distance between two connected
	 *            RigidPRMWaypoints to set
	 */
	public void setMaxDistance(double maxDistance) {
		this.maxDistance = maxDistance;
	}

	/**
	 * Checks if the optimal expression for maximum neighbors is active or not.
	 * 
	 * @return true if the expression is active, false otherwise
	 */
	public boolean isOptimalMaxNeighbors() {
		return optimalMaxNeighbors;
	}

	/**
	 * Sets the status for the optimal expression for maximum neighbors.
	 * 
	 * @param optimalMaxNeighbors the status of the optimal expression for maximum
	 *            neighbors to set
	 */
	public void setOptimalMaxNeighbors(boolean optimalMaxNeighbors) {
		this.optimalMaxNeighbors = optimalMaxNeighbors;
	}

	/**
	 * Checks if the optimal expression for maximum distance is active or not.
	 * 
	 * @return true if the expression is active, false otherwise
	 */
	public boolean isOptimalMaxDistance() {
		return optimalMaxDistance;
	}

	/**
	 * Sets the status for the optimal expression for maximum distance.
	 * 
	 * @param optimalMaxDistance the status of the optimal expression for maximum
	 *            neighbors to set
	 */
	public void setOptimalMaxDistance(boolean optimalMaxDistance) {
		this.optimalMaxDistance = optimalMaxDistance;
	}

	/**
	 * Gets the sampling strategy.
	 * 
	 * @return the samplingStrategy used to create waypoints
	 */
	public Sampling getSamplingStrategy() {
		return samplingStrategy;
	}

	/**
	 * Sets the sampling strategy.
	 * 
	 * @param samplingStrategy the samplingStrategy to set
	 */
	public void setSamplingStrategy(Sampling samplingStrategy) {
		this.samplingStrategy = samplingStrategy;
	}

	/**
	 * Gets the enhancement mode or weight function.
	 * 
	 * @return the enhancement mode or weight function
	 */
	public EnhancementMode getEnhancement() {
		return enhancement;
	}

	/**
	 * Sets the enhancement mode or weight function.
	 * 
	 * @param enhancement the enhancement mode or weight function to set
	 */
	public void setEnhancement(EnhancementMode enhancement) {
		this.enhancement = enhancement;
	}

	/**
	 * Gets the collision delay method.
	 * 
	 * @return the delayCollision the collision delay method.
	 */
	public CollisionDelay getDelayCollision() {
		return delayCollision;
	}

	/**
	 * Sets the collision delay method.
	 * 
	 * @param delayCollision the collision delay method to set
	 */
	public void setDelayCollision(CollisionDelay delayCollision) {
		this.delayCollision = delayCollision;
	}

	/**
	 * Gets the minimum quality (initial inflation) of this Rigid PRM properties
	 * bean.
	 * 
	 * @return the minimum quality (initial inflation) of this Rigid PRM properties
	 *         bean
	 * 
	 * @see AnytimePlannerProperties#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.minimumQuality;
	}

	/**
	 * Sets the minimum quality (initial inflation) of this Rigid PRM properties
	 * bean.
	 * 
	 * @param minimumQuality the minimum quality (initial inflation) of this Rigid
	 *            PRM properties bean
	 * 
	 * @throws IllegalArgumentException if the initial inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double minimumQuality) {
		if ((1d <= minimumQuality) &&
				(minimumQuality >= this.maximumQuality)) {
			this.minimumQuality = minimumQuality;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}
	
	/**
	 * Gets the maximum quality (final inflation) of this Rigid PRM properties bean.
	 * 
	 * @return the maximum quality (final inflation) of this Rigid PRM properties
	 *         bean
	 * 
	 * @see AnytimePlannerProperties#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.maximumQuality;
	}

	/**
	 * Sets the maximum quality (initial inflation) of this Rigid PRM properties
	 * bean.
	 * 
	 * @param maximumQuality the maximum quality (final inflation) of this Rigid PRM
	 *            properties bean
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double maximumQuality) {
		if ((1d <= maximumQuality) && (this.minimumQuality >= maximumQuality)) {
			this.maximumQuality = maximumQuality;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}

	/**
	 * Gets the quality improvement (deflation amount) of this Rigid PRM properties
	 * bean.
	 * 
	 * @return the quality improvement (deflation amount) of this Rigid PRM
	 *         properties bean
	 * 
	 * @see AnytimePlannerProperties#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.qualityImprovement;
	}

	/**
	 * Sets the quality improvement (deflation amount) of this Rigid PRM properties
	 * bean.
	 * 
	 * @param qualityImprovement the quality improvement (deflation amount) of this
	 *            Rigid PRM properties bean
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlannerProperties#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double qualityImprovement) {
		if (0d < qualityImprovement) {
			this.qualityImprovement = qualityImprovement;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}
	
	/**
	 * Gets the query planner of this planner.
	 * 
	 * @return the planner used to find a path in this environment.
	 */
	public QueryPlanner getPlanner() {
		return planner;
	}

	/**
	 * Sets the query planner of this planner.
	 * 
	 * @param planner the planner to set
	 */
	public void setPlanner(QueryPlanner planner) {
		this.planner = planner;
	}

	/**
	 * Gets the query mode of this planner.
	 * 
	 * @return the mode the query mode
	 */
	public QueryMode getMode() {
		return mode;
	}

	/**
	 * Sets the query mode of this planner.
	 * 
	 * @param mode the mode to set
	 */
	public void setMode(QueryMode mode) {
		this.mode = mode;
	}
}
