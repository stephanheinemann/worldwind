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

/**
 * Realizes the properties bean of a FAD PRM planner.
 * 
 * @author Henrique Ferreira
 *
 */
public class FADPRMProperties extends AbstractPlannerProperties implements AnytimePlannerProperties {

	/** the maximum number of neighbors a waypoint can be connected to */
	private int maxNeighbors = 15;

	/** the maximum distance between two neighboring waypoints */
	private double maxDistance = 400d;

	/** the initial inflation factor (initial beta value) */
	private double initialInflation = 0d;

	/** the final inflation factor (final beta value) */
	private double finalInflation = 1d;

	/** the inflation amount to be applied to the current inflation */
	private double inflationAmount = 0.1;

	/** the bias of the sampling algorithm towards goal */
	private int bias = 5;

	/**
	 * Constructs a new FAD PRM planner properties bean.
	 */
	public FADPRMProperties() {
		super();
		this.setMinimumQuality(0d);
		this.setMaximumQuality(1d);
		this.setQualityImprovement(0.1);
	}

	/**
	 * Gets the maximum number of neighbors a waypoint can have
	 * 
	 * @return the maximum number of neighbors a waypoint can have
	 */
	public int getMaxNeighbors() {
		return maxNeighbors;
	}

	/**
	 * Sets the maximum number of neighbors a waypoint can have
	 * 
	 * @param maxNeighbors the maximum number of neighbors a waypoint can have
	 */
	public void setMaxNeighbors(int maxNeighbors) {
		this.maxNeighbors = maxNeighbors;
	}

	/**
	 * Gets the maximum distance between two connected waypoints
	 * 
	 * @return the maximum distance between two connected waypoints
	 */
	public double getMaxDistance() {
		return maxDistance;
	}

	/**
	 * Sets the maximum distance between two connected waypoints
	 * 
	 * @param maxDistance maximum distance between two connected waypoints
	 */
	public void setMaxDistance(double maxDistance) {
		this.maxDistance = maxDistance;
	}

	/**
	 * Gets the minimum quality (initial inflation) of this FAD PRM properties bean.
	 * 
	 * @return the minimum quality (initial inflation) of this FAD PRM properties
	 *         bean
	 * 
	 * @see AnytimePlannerProperties#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialInflation;
	}

	/**
	 * Sets the minimum quality (initial inflation) of this FAD PRM properties bean.
	 * 
	 * @param initialInflation the minimum quality (initial inflation) of this FAD
	 *            PRM properties bean
	 * 
	 * @throws IllegalArgumentException if the initial inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialInflation) {
		if ((0d <= initialInflation) &&
				(initialInflation <= this.finalInflation)) {
			this.initialInflation = initialInflation;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}

	/**
	 * Gets the maximum quality (final inflation) of this FAD PRM properties bean.
	 * 
	 * @return the maximum quality (final inflation) of this FAD PRM properties bean
	 * 
	 * @see AnytimePlannerProperties#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalInflation;
	}

	/**
	 * Sets the maximum quality (final inflation) of this FAD PRM properties bean.
	 * 
	 * @param finalInflation the maximum quality (final inflation) of this FAD PRM
	 *            properties bean
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalInflation) {
		if ((1d >= finalInflation) && (this.initialInflation <= finalInflation)) {
			this.finalInflation = finalInflation;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}

	/**
	 * Gets the quality improvement (inflation amount) of this FAD PRM properties
	 * bean.
	 * 
	 * @return the quality improvement (inflation amount) of this FAD PRM properties
	 *         bean
	 * 
	 * @see AnytimePlannerProperties#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.inflationAmount;
	}

	/**
	 * Sets the quality improvement (inflation amount) of this FAD PRM properties
	 * bean.
	 * 
	 * @param inflationAmount the quality improvement (inflation amount) of this FAD
	 *            PRM properties bean
	 * 
	 * @throws IllegalArgumentException if the inflation amount is invalid
	 * 
	 * @see AnytimePlannerProperties#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double inflationAmount) {
		if (0d < inflationAmount) {
			this.inflationAmount = inflationAmount;
		} else {
			throw new IllegalArgumentException("inflation amount is invalid");
		}
	}

	/**
	 * Gets the bias of sampling towards the goal of this FAD PRM properties bean.
	 * 
	 * @return the bias of sampling towards the goal
	 */
	public int getBias() {
		return bias;
	}

	/**
	 * Sets the bias of sampling towards the goal of this FAD PRM properties bean.
	 * 
	 * @param the bias of sampling towards the goal
	 */
	public void setBias(int bias) {
		this.bias = bias;
	}
}
