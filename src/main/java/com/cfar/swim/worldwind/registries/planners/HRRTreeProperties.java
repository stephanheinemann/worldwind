/**
 * Copyright (c) 2018, Manuel Rosa (UVic Center for Aerospace Research)
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

import com.cfar.swim.worldwind.ai.rrt.hrrt.Heuristic;

/**
 * Realizes the properties bean of an heuristic RRTree planner.
 * 
 * @author Manuel Rosa
 *
 */
public class HRRTreeProperties extends RRTreeProperties {
	
	/** the description of this planner properties bean */
	private final static String DESCRIPTION_HRRT = "Heuristic RRT: Heuristic version of a RRT which takes into account the cost of a given position when selecting the next position to expand. It considers the costs associated with traversable obstacles.";
	
	/** floor value to ensure the search is not overly biased against exploration */
	private double probFloor = 0.9;
	
	/** the number of neighbors to consider as parent for the sampled waypoint  */
	private int neighbors = 5;
	
	/** the heuristic algorithm for the planner */
	private Heuristic heuristic = Heuristic.BkRRT;
	
	/**
	 * Constructs a new heuristic RRTree planner properties bean.
	 */
	public HRRTreeProperties() {
		super();
		this.setDescription(DESCRIPTION_HRRT);
	}

	/**
	 * Gets the heuristic algorithm for the planner
	 * 
	 * @return the heuristic algorithm to set
	 */
	public Heuristic getHeuristic() {
		return heuristic;
	}

	/**
	 * Sets the heuristic algorithm for the planner
	 * 
	 * @param heuristic the heuristic algorithm to set
	 */
	public void setHeuristic(Heuristic heuristic) {
		this.heuristic = heuristic;
	}

	/**
	 * Gets the probability floor for the acceptance of a sampled waypoint
	 * 
	 * @return the probability floor
	 */
	public double getProbFloor() {
		return probFloor;
	}

	/**
	 * Sets the probability floor for the acceptance of a sampled waypoint
	 * 
	 * @param probFloor the probability floor
	 */
	public void setProbFloor(double probFloor) {
		this.probFloor = probFloor;
	}
	
	/**
	 * Gets number of neighbors to consider to test a sampled waypoint
	 * 
	 * @return the number of neighbors
	 */
	public int getNeighbors() {
		return neighbors;
	}

	/**
	 * Sets number of neighbors to consider to test a sampled waypoint
	 * 
	 * @param neighbors the number of neighbors
	 */
	public void setNeighbors(int neighbors) {
		this.neighbors = neighbors;
	}
	

	/** defines which formula to use for quality calculation */
	public boolean myQuality = true;
	
	/** defines which formula to use for probability calculation */
	public boolean myProbability = true;

	/**
	 * @return the myQuality
	 */
	public boolean isMyQuality() {
		return myQuality;
	}

	/**
	 * @param myQuality the myQuality to set
	 */
	public void setMyQuality(boolean myQuality) {
		this.myQuality = myQuality;
	}

	/**
	 * @return the myProbability
	 */
	public boolean isMyProbability() {
		return myProbability;
	}

	/**
	 * @param myProbability the myProbability to set
	 */
	public void setMyProbability(boolean myProbability) {
		this.myProbability = myProbability;
	}
	
	
	
}
