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
package com.cfar.swim.worldwind.ai.prm.lazyprm;

import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRMConstructor;
import com.cfar.swim.worldwind.planning.Environment;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a Lazy PRM that constructs a Planning Roadmap by sampling points in
 * a continuous environment without taking terrain obstacles into account.
 * 
 * @author Henrique Ferreira
 *
 */
public class LazyPRMConstructor extends BasicPRMConstructor {

	/**
	 * Constructs a Lazy PRM for a specified environment, a maximum number of
	 * iterations (Positions), a maximum number of neighbors (of a single Position)
	 * and a maximum distance between two connected neighbors.
	 * 
	 * @param environment the environment
	 * @param maxIter the maximum number of iterations
	 * @param maxNeighbors the maximum number of neighbors
	 * @param maxDist the maximum distance between two connected neighbors
	 */
	public LazyPRMConstructor(Environment environment, int maxIter, int maxNeighbors, double maxDist) {
		super(environment, maxIter, maxNeighbors, maxDist);
	}

	/**
	 * Connects this position to another positions already sampled, checking if the
	 * two positions are connectable. Then, creates a new planning roadmap based on
	 * the two positions.
	 * 
	 * @param position the position to be connected
	 *
	 * @see com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRMConstructor#connectPosition(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	protected void connectPosition(Position position) {
		int numConnectedNeighbor = 0;

		this.sortNearest(position);

		for (Position neighbor : this.getPositionList()) {
			if (this.canConnectPositions(position, neighbor, numConnectedNeighbor)) {
				this.getEnvironment().addChild(position, neighbor);
				numConnectedNeighbor++;
			}
		}
	}

	/**
	 * Creates the roadmap by sampling positions from a continuous environment. Does
	 * not check if the position has conflicts with terrain. Then, adds the position
	 * to positions list and tries to connect to other already sampled positions.
	 * 
	 * @see com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRMConstructor#construct()
	 */
	@Override
	public void construct() {
		int num = 0;

		this.getAllPositions();

		while (num < this.maxIter) {
			Position position = this.getEnvironment().sampleRandomPosition();
			this.positionList.add(position);
			this.connectPosition(position);
			num++;
		}

	}
}
