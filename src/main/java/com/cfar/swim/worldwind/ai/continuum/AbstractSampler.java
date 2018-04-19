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
package com.cfar.swim.worldwind.ai.continuum;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.render.TerrainObstacle;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;

/**
 * Abstracts a sampling based motion planner for an aircraft in an environment
 * using cost and risk policies.
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public abstract class AbstractSampler extends AbstractPlanner implements Sampler {

	/** the list of already sampled waypoints */
	private List<SampledWaypoint> waypointList = new ArrayList<SampledWaypoint>();

	/** the environment casted to a planning continuum */
	private PlanningContinuum continuumEnvironment = null;

	/**
	 * Constructs a sampling based motion planner with a specified aircraft and
	 * environment.
	 * 
	 * @param aircraft the aircraft for planning
	 * @param environment the environment for planning
	 */
	public AbstractSampler(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		if (this.getEnvironment() instanceof PlanningContinuum) {
			this.setContinuumEnvironment((PlanningContinuum) this.getEnvironment());
		}
	}

	/**
	 * Gets the list of already sampled waypoints
	 * 
	 * @return the waypointList
	 */
	public List<SampledWaypoint> getWaypointList() {
		return waypointList;
	}

	/**
	 * Sets the list of waypoints previously sampled
	 * 
	 * @param waypointList the waypointList to set
	 */
	public void setWaypointList(List<SampledWaypoint> waypointList) {
		this.waypointList = waypointList;
	}

	/**
	 * Gets the environment of this abstract sampler.
	 * 
	 * @return the continuum environment of this abstract sampler
	 */
	public PlanningContinuum getContinuumEnvironment() {
		return continuumEnvironment;
	}

	/**
	 * Sets the environment of this abstract sampler.
	 * 
	 * @param continuumEnvironment the continuum environment of this abstract
	 *            sampler
	 */
	public void setContinuumEnvironment(PlanningContinuum continuumEnvironment) {
		this.continuumEnvironment = continuumEnvironment;
	}

	/**
	 * Samples a position from a continuous space defined in the current environment
	 * 
	 * @return position in global coordinates inside the environment
	 */
	public Position sampleRandomPosition() {
		Vec4[] corners = this.getContinuumEnvironment().getCorners();

		// Point in box frame with all minimum coordinates
		Vec4 minimum = this.getContinuumEnvironment().transformModelToBoxOrigin(corners[0]);
		// Point in box frame with all maximum coordinates
		Vec4 maximum = this.getContinuumEnvironment().transformModelToBoxOrigin(corners[6]);

		double x, y, z;

		x = minimum.x + (new Random().nextDouble() * (maximum.x - minimum.x));
		y = minimum.y + (new Random().nextDouble() * (maximum.y - minimum.y));
		z = minimum.z + (new Random().nextDouble() * (maximum.z - minimum.z));

		Vec4 point = new Vec4(x, y, z);

		// Transform point from box frame to earth frame
		point = this.getContinuumEnvironment().transformBoxOriginToModel(point);

		Position position = this.getEnvironment().getGlobe().computePositionFromPoint(point);

		return position;
	}

	/**
	 * Creates a SampledWaypoint embedding the CostInterval tree for respective
	 * position
	 * 
	 * @param position the position for the new waypoint
	 * 
	 * @return the SampledWaypoint with position and CostInterval
	 */
	public SampledWaypoint createSampledWaypoint(Position position) {
		SampledWaypoint sampledWaypoint = new SampledWaypoint(position);

		sampledWaypoint.setCostIntervals(this.getContinuumEnvironment().getIntervalTree(position));

		return sampledWaypoint;

	}

	/**
	 * Checks whether the given position is inside the given globe
	 * 
	 * @param globe the globe
	 * @param position the position in global coordinates
	 * @return true if the position is inside the globe and false otherwise
	 */
	public boolean isInsideGlobe(Globe globe, Position position) {
		Vec4 point;

		point = this.getContinuumEnvironment().getGlobe().computePointFromPosition(position);
		return !globe.isPointAboveElevation(point, globe.getElevation(position.latitude, position.longitude));
	}

	/**
	 * Checks if a given position is in conflict with untraversable obstacles in the
	 * environment
	 * 
	 * @param waypoint the waypoint in global coordinates
	 * 
	 * @return boolean value true if there is a conflict
	 */
	public boolean checkConflict(Position position) {

		if (this.isInsideGlobe(this.getContinuumEnvironment().getGlobe(), position))
			return true;

		// TODO : Implement a checker for conflict between a position and the
		// static, time independent and untraversable obstacles in the environment

		Box box = this.getContinuumEnvironment().createBoundingBox(position);
		HashSet<TerrainObstacle> terrainSet =
				this.getContinuumEnvironment().getTerrainObstacles();
		for(TerrainObstacle terrain : terrainSet) {
			// // Check if obstacle contains the waypoint
			System.out.println("Terrain");
			if(terrain.getExtent(this.getContinuumEnvironment().getGlobe()).intersects(box.getFrustum())){
				return true;
			}
		}
		return false;
	}

	/**
	 * Checks if a straight leg between the waypoints is in conflict with
	 * untraversable obstacles in the environment
	 * 
	 * @param waypoint1 the first waypoint in global coordinates
	 * @param waypoint2 the second waypoint in global coordinates
	 * 
	 * @return boolean value true if there is a conflict
	 */
	public boolean checkConflict(Position position1, Position position2) {
		Vec4 point1 = this.getContinuumEnvironment().getGlobe().computePointFromPosition(position1);
		Vec4 point2 = this.getContinuumEnvironment().getGlobe().computePointFromPosition(position2);
		Vec4 aux;

		double x, y, z, dx, dy, dz, dist, dist2, theta, phi;

		dx = point2.x - point1.x;
		dy = point2.y - point1.y;
		dz = point2.z - point1.z;

		dist = point1.distanceTo3(point2);
		dist2 = point1.distanceTo2(point2);
		theta = Math.atan2(dz, dist2);
		phi = Math.atan2(dy, dx);

		double resolution = this.getContinuumEnvironment().getResolution(); // meters in globe surface
		for (int p = 1; dist > resolution; p = p * 2) {
			for (int k = 0; k < p; k++) {
				x = point1.x + (1 / 2 + k) * dist * Math.cos(theta) * Math.cos(phi);
				y = point1.y + (1 / 2 + k) * dist * Math.cos(theta) * Math.sin(phi);
				z = point1.z + (1 / 2 + k) * dist * Math.sin(theta);
				aux = new Vec4(x, y, z);
				if (this.checkConflict(this.getContinuumEnvironment().getGlobe().computePositionFromPoint(aux)))
					return true;
			}
			dist = dist / 2;
		}

		return false;
	}

	/**
	 * Finds the k-nearest waypoints to the given position
	 * 
	 * @param position the position in global coordinates
	 * @param kNear number of waypoints to return
	 * 
	 * @return list of k-nearest waypoints sorted by increasing distance
	 */
	public List<? extends Position> findNearest(Position position, int kNear) {
		List<? extends Position> posiNearList;

		// sorts the list by increasing distance to waypoint
		this.sortNearest(position);

		// If there are less than k neighbors on the list
		if (this.waypointList.size() <= kNear) {
			posiNearList = new ArrayList<>(this.waypointList);
		}
		// If there are more than k neighbors on the list
		else {
			posiNearList = new ArrayList<>(this.waypointList.subList(0, kNear));
		}

		return posiNearList;
	}

	/**
	 * Sorts a list of elements by increasing distance to a given position
	 * 
	 * @param position the position in global coordinates
	 */
	public void sortNearest(Position position) {

		Collections.sort(this.waypointList,
				(a, b) -> super.getEnvironment().getDistance(position, a) < super.getEnvironment().getDistance(position,
						b) ? -1
								: super.getEnvironment().getDistance(position, a) == super.getEnvironment()
										.getDistance(position, b) ? 0 : 1);

	}
}
