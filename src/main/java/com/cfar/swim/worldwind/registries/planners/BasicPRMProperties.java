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

import com.cfar.swim.worldwind.ai.prm.basicprm.QueryMode;
import com.cfar.swim.worldwind.ai.prm.basicprm.QueryPlanner;

/**
 * Realizes the properties bean of a basic PRM planner.
 * 
 * @author Henrique Ferreira
 *
 */
public class BasicPRMProperties extends AbstractPlannerProperties implements AnytimePlannerProperties {

	/** the description of this planner properties bean */
	private final static String DESCRIPTION_BASICPRM = "Probabilistic Roadmap: Basic version of a PRM that constructs a roadmap by sampling points in a continuous environment. Then, a deterministic planner is used to compute the desired trajectory.";

	/** the maximum number of sampling iterations */
	private int maxIter = 300;

	/** the maximum number of neighbors a waypoint can be connected to */
	private int maxNeighbors = 20;

	/** the maximum distance between two neighboring waypoints */
	private double maxDistance = 200;

	/** the query planner of this PRM planner properties bean */
	private QueryPlanner queryPlanner = QueryPlanner.FAS;

	/** the query mode of this basic PRM planner properties bean */
	private QueryMode mode = QueryMode.SINGLE;

	/** the initial inflation factor applied to the heuristic function */
	private double minimumQuality = 50d;

	/** the final inflation factor applied to the heuristic function */
	private double maximumQuality = 1d;

	/** the inflation amount to be applied to the current inflation */
	private double qualityImprovement = 1d;

	/**
	 * Constructs a new basic PRM planner properties bean.
	 */
	public BasicPRMProperties() {
		super();
		this.setDescription(DESCRIPTION_BASICPRM);
	}

	/**
	 * Gets the maximum number of sampling iterations.
	 * 
	 * @return the maximum number of sampling iterations
	 */
	public int getMaxIter() {
		return maxIter;
	}

	/**
	 * Sets the maximum number of sampling iterations.
	 * 
	 * @param maxIter the maximum number of sampling iterations
	 */
	public void setMaxIter(int maxIter) {
		this.maxIter = maxIter;
	}

	/**
	 * Gets the maximum number of neighbors a waypoint can have.
	 * 
	 * @return the maximum number of neighbors a waypoint can have
	 */
	public int getMaxNeighbors() {
		return maxNeighbors;
	}

	/**
	 * Sets the maximum number of neighbors a waypoint can have.
	 * 
	 * @param maxNeighbors the maximum number of neighbors a waypoint can have
	 */
	public void setMaxNeighbors(int maxNeighbors) {
		this.maxNeighbors = maxNeighbors;
	}

	/**
	 * Gets the maximum distance between two connected waypoints.
	 * 
	 * @return the maximum distance between two connected waypoints
	 */
	public double getMaxDistance() {
		return maxDistance;
	}

	/**
	 * Sets the maximum distance between two connected waypoints.
	 * 
	 * @param maxDistance maximum distance between two connected waypoints
	 */
	public void setMaxDistance(double maxDistance) {
		this.maxDistance = maxDistance;
	}

	/**
	 * Gets the query planner of this planner.
	 * 
	 * @return the queryPlanner the planner used to find a path in the roadmap
	 */
	public QueryPlanner getQueryPlanner() {
		return queryPlanner;
	}

	/**
	 * Sets the query planner of this planner.
	 * 
	 * @param queryPlanner the queryPlanner to set
	 */
	public void setQueryPlanner(QueryPlanner queryPlanner) {
		this.queryPlanner = queryPlanner;
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

	/**
	 * Gets the minimum quality (initial inflation) of this Basic PRM properties
	 * bean.
	 * 
	 * @return the minimum quality (initial inflation) of this Basic PRM properties
	 *         bean
	 * 
	 * @see AnytimePlannerProperties#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.minimumQuality;
	}

	/**
	 * Sets the minimum quality (initial inflation) of this Basic PRM properties
	 * bean.
	 * 
	 * @param minimumQuality the minimum quality (initial inflation) of this Basic
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
	 * Gets the maximum quality (final inflation) of this Basic PRM properties bean.
	 * 
	 * @return the maximum quality (final inflation) of this Basic PRM properties
	 *         bean
	 * 
	 * @see AnytimePlannerProperties#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.maximumQuality;
	}

	/**
	 * Sets the maximum quality (initial inflation) of this Basic PRM properties
	 * bean.
	 * 
	 * @param maximumQuality the maximum quality (final inflation) of this Basic PRM
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
	 * Gets the quality improvement (deflation amount) of this Basic PRM properties
	 * bean.
	 * 
	 * @return the quality improvement (deflation amount) of this Basic PRM
	 *         properties bean
	 * 
	 * @see AnytimePlannerProperties#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.qualityImprovement;
	}

	/**
	 * Sets the quality improvement (deflation amount) of this Basic PRM properties
	 * bean.
	 * 
	 * @param qualityImprovement the quality improvement (deflation amount) of this
	 *            Basic PRM properties bean
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
}
