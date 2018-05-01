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
package com.cfar.swim.worldwind.ai.prm.basicprm;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningRoadmap;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a basic PRM that constructs a Planning Roadmap by sampling points in
 * a continuous environment.
 * 
 * @author Henrique Ferreira
 *
 */
public class BasicPRM {

	/** the maximum number of sampling iterations */
	protected int maxIter;

	/** the maximum number of neighbors a position can be connected to */
	protected int maxNeighbors;

	/** the maximum distance between two neighboring sampled positions */
	protected double maxDist;

	/** the environment of this basic PRM */
	protected Environment environment = null;

	/** the list of already sampled positions */
	protected List<Position> positionList = new ArrayList<Position>();

	/**
	 * Constructs a basic PRM for a specified environment, a maximum number of
	 * iterations (Positions), a maximum number of neighbors (of a single Position)
	 * and a maximum distance between two connected neighbors.
	 * 
	 * @param environment the environment
	 * @param maxIter the maximum number of iterations
	 * @param maxNeighbors the maximum number of neighbors
	 * @param maxDist the maximum distance between two connected neighbors
	 */
	public BasicPRM(Environment environment, int maxIter, int maxNeighbors, double maxDist) {
		this.environment = environment;
		this.maxIter = maxIter;
		this.maxNeighbors = maxNeighbors;
		this.maxDist = maxDist;
	}

	/**
	 * Gets the planning roadmap of this planner
	 * 
	 * @return the planning roadmap
	 */
	public PlanningRoadmap getEnvironment() {
		return (PlanningRoadmap) environment;
	}

	/**
	 * Gets the list of already sampled positions
	 * 
	 * @return the list of positions
	 */
	public List<Position> getPositionList() {
		return positionList;
	}

	/**
	 * Sets the list of positions previously sampled
	 * 
	 * @param positionList the list of positions to set
	 * 
	 */
	public void setPositionList(List<Position> positionList) {
		this.positionList = positionList;
	}

	/**
	 * Connects this position to another positions already sampled, checking if the
	 * two positions are connectable. Then, creates a new planning roadmap based on
	 * the two positions.
	 * 
	 * @param position the position to be connected
	 */
	protected void connectPosition(Position position) {
		int numConnectedNeighbor = 0;

		this.sortNearest(position);

		for (Position neighbor : this.getPositionList()) {
			if (this.canConnectPositions(position, neighbor, numConnectedNeighbor)) {
				if(!this.getEnvironment().checkConflict(neighbor, position)) {
					this.getEnvironment().addChild(position, neighbor);
					numConnectedNeighbor++;
				}
			}

		}
	}

	/**
	 * Indicates whether or not the two positions are connectable. In order to be
	 * connectable the two positions must be closer than a maximum distance and the
	 * number of connected neighbors must be less than a defined maximum number of
	 * neighbors.
	 * 
	 * @param position the position in globe coordinates
	 * @param neighbor the neighbor position in globe coordinates
	 * @param num the number of connected neighbors
	 * @return true if the two positions are connectable, false otherwise
	 */
	protected boolean canConnectPositions(Position position, Position neighbor, int num) {
		boolean connectable = false;

		if (environment.getDistance(neighbor, position) < this.maxDist && num < this.maxNeighbors) {
			connectable = true;
		}

		return connectable;
	}

	/**
	 * Creates the roadmap by sampling positions from a continuous environment.
	 * Checks if the position has conflicts with terrain. Then, adds the position to
	 * positions list and tries to connect to other already sampled positions.
	 */
	public void construct() {
		int num = 0;

		this.getAllPositions();

		while (num < this.maxIter) {
			Position position = this.getEnvironment().sampleRandomPosition();

			if (!this.getEnvironment().checkConflict(position)) {
				this.positionList.add(position);
				this.connectPosition(position);
				num++;
			}
		}
	}

	/**
	 * Gets all the positions associated with this environment. Adds the origin and
	 * the 3D opposite corner of the origin of all environments to the positions
	 * list.
	 */
	protected void getAllPositions() {
		// TODO: review if code below is working correctly
		for (PlanningRoadmap env : this.getEnvironment().getAll()) {
			if (env.hasParent()) {
				this.positionList.add(env.getGlobe().computePositionFromPoint(env.getOrigin()));
				this.positionList.add(env.getGlobe().computePositionFromPoint(env.get3DOpposite()));
			}
		}
		this.setPositionList(this.getPositionList().stream().distinct().collect(Collectors.toList()));

	}

	/**
	 * Finds the k-nearest positions to the given position
	 * 
	 * @param position the position in global coordinates
	 * @param kNear number of positions to return
	 * 
	 * @return list of k-nearest positions sorted by increasing distance
	 */
	public List<Position> findNearest(Position position, int kNear) {

		return this.getPositionList().stream()
				.sorted((p1, p2) -> Double.compare(this.getEnvironment().getNormalizedDistance(p1, position),
						this.getEnvironment().getNormalizedDistance(p2, position)))
				.limit(kNear).collect(Collectors.toList());

	}

	/**
	 * Sorts a list of elements by increasing distance to a given position
	 * 
	 * @param position the position in global coordinates
	 */
	public void sortNearest(Position position) {

		this.setPositionList(
				this.getPositionList().stream()
						.sorted((p1, p2) -> Double.compare(this.getEnvironment().getNormalizedDistance(p1, position),
								this.getEnvironment().getNormalizedDistance(p2, position)))
						.collect(Collectors.toList()));

	}

	/**
	 * Indicates whether or not this basic PRM supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is supported, false otherwise
	 */
	public boolean supports(Environment environment) {
		boolean supports = false;
		if (null != environment)
			supports = true;

		if (supports) {
			supports = (environment instanceof PlanningRoadmap);
		}

		return supports;
	}
}
