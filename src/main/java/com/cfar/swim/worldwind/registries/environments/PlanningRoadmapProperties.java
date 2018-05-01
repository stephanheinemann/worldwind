/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.registries.environments;

import com.cfar.swim.worldwind.planning.RoadmapConstructor;

/**
 * Realizes the properties bean of a planning roadmap environment.
 * 
 * @author Stephan Heinemann
 *
 */
public class PlanningRoadmapProperties extends SamplingEnvironmentProperties {
	
	/** the algorithm used to construct the roadmap of this Planning Roadmap properties bean */
	RoadmapConstructor roadmapConstructor = RoadmapConstructor.BASICPRM;
	
	/** the maximum number of sampling iterations */
	private int maxIter = 300;
	
	/** the maximum number of neighbors a waypoint can be connected to */
	private int maxNeighbors = 30;
	
	/** the maximum distance between two neighboring waypoints */
	private double maxDistance = 200;
	
	public PlanningRoadmapProperties() {
		super();
	}
	
	public PlanningRoadmapProperties(int maxIter, int maxNeighbors, double maxDistance) {
		super();
		this.setMaxIter(maxIter);
		this.setMaxNeighbors(maxNeighbors);
		this.setMaxDistance(maxDistance);
	}
	
	/**
	 * Gets the constructor of this Planning Roadmap properties bean.
	 * 
	 * @return the constructor of this Planning Roadmap properties bean
	 */
	public RoadmapConstructor getRoadmapConstructor() {
		return this.roadmapConstructor;
	}
	
	/**
	 * Sets the constructor of this Planning Roadmap properties bean.
	 * 
	 * @param roadmapConstructor the constructor to be set
	 */
	public void setRoadmapConstructor(RoadmapConstructor roadmapConstructor) {
		this.roadmapConstructor = roadmapConstructor;
	}
	
	/**
	 * Gets the maximum number of sampling iterations
	 * 
	 * @return the maximum number of sampling iterations
	 */
	public int getMaxIter() {
		return maxIter;
	}

	/**
	 * Sets the maximum number of sampling iterations
	 * 
	 * @param maxIter the maximum number of sampling iterations
	 */
	public void setMaxIter(int maxIter) {
		this.maxIter = maxIter;
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
}
