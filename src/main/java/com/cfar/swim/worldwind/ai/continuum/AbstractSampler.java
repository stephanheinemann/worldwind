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
import java.util.List;
import java.util.Random;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Abstracts a sampling based motion planner for an aircraft in an environment
 * using cost and risk policies.
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public abstract class AbstractSampler extends AbstractPlanner
		implements Sampler {

	// TODO: Should this be defined here or specifically for every algorithm?
	// (Probably not here...)
	/** the list of already sampled waypoints */
	private List<? extends SampledWaypoint> waypointList = null;

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
			this.setContinuumEnvironment(
					(PlanningContinuum) this.getEnvironment());
		}
	}

	/**
	 * Gets the list of already sampled waypoints
	 * 
	 * @return the waypointList
	 */
	public List<? extends SampledWaypoint> getWaypointList() {
		return waypointList;
	}

	/**
	 * Sets the list of waypoints previously sampled
	 * 
	 * @param waypointList the waypointList to set
	 */
	public void setWaypointList(List<? extends SampledWaypoint> waypointList) {
		this.waypointList = waypointList;
	}

	/**
	 * @return the continuumEnvironment
	 */
	public PlanningContinuum getContinuumEnvironment() {
		return continuumEnvironment;
	}

	/**
	 * @param continuumEnvironment the continuumEnvironment to set
	 */
	public void setContinuumEnvironment(
			PlanningContinuum continuumEnvironment) {
		this.continuumEnvironment = continuumEnvironment;
	}

	/**
	 * Samples a position from a continuous space defined in the current
	 * environment
	 * 
	 * @return position in global coordinates inside the environment
	 */
	public Position sampleRandomPosition() {
		Vec4[] corners = this.getContinuumEnvironment().getCorners();
		
		// Point in box with all minimum coordinates
		Vec4 minimum = this.getContinuumEnvironment().transformModelToBoxOrigin(corners[0]); 

		// Point in box with all maximum coordinates
		Vec4 maximum = this.getContinuumEnvironment().transformModelToBoxOrigin(corners[6]); 
		
		double x, y, z;

		x = minimum.x + (new Random().nextDouble() * (maximum.x - minimum.x));
		y = minimum.y + (new Random().nextDouble() * (maximum.y - minimum.y));
		z = minimum.z + (new Random().nextDouble() * (maximum.z - minimum.z));	
		
		Vec4 point = new Vec4(x, y, z);
		
		point= this.getContinuumEnvironment().transformBoxOriginToModel(point);

		Position position = this.getEnvironment().getGlobe()
				.computePositionFromPoint(point);

		return position;
	}

	/**
	 * Checks if a given position is in conflict with untraversable obstacles in
	 * the environment
	 * 
	 * @param waypoint the waypoint in global coordinates
	 * 
	 * @return boolean value true if there is a conflict
	 */
	public boolean checkConflict(Position position) {
		// TODO : Implement a checker for conflict between a position and the
		// static, time independent and untraversable obstacles in the environment
//		
//		HashSet<Terrain> terrainSet = this.getContinuumEnvironment().getTerrain();
//		
//		for(Terrain terrain : terrainSet) {
//			// Check if obstacle contains the waypoint
//			if(terrain.getExtent(this.getContinuumEnvironment().getGlobe()).contains(position)){
//				return true;
//			}
//		}
//		
		return false;
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
		
		sampledWaypoint.setCostIntervals( this.getContinuumEnvironment().getIntervalTree(position) );
		
		return sampledWaypoint;
		
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
		Position positionAux;
		double lat, lon, elevation, angle, ddLat, ddLon, ddElevation;
		
		ddLat = position2.getLatitude().radians - position1.getLatitude().radians;
		ddLon = position2.getLongitude().radians - position1.getLongitude().radians;
		ddElevation = position2.getElevation() - position1.getElevation();
		
		angle = Math.acos( Math.cos(ddLon)*Math.cos(ddLat) ); // angle (radians) between 2 positions and globe center
		
		// TODO: add resolution as user input parameter in UI
		double RESOLUTION=1; //meters in globe surface
		double GLOBE_RADIUS = this.getEnvironment().getGlobe().getEquatorialRadius();
		// TODO: Review if angles used are correct
		for(int p=1; angle*GLOBE_RADIUS>RESOLUTION; p=p*2){
			for(int k=0; k<p; k++){
				// latitude with fraction of angle between positions
				lat = Math.asin( Math.sin((1/2 + k)*angle) * Math.sin(ddLat) );
				// longitude using spherical pythagoras theorem
				lon = Math.acos( Math.cos((1/2 + k)*angle) / Math.cos(lat) );
				// elevation adding linear fraction of elevation variation
				elevation = position1.getElevation() + (1/2 + k) * ddElevation;
				
				positionAux = new Position(Angle.fromDegrees(lat), Angle.fromDegrees(lon), elevation);
						
				if(this.checkConflict(positionAux)) {
					return true;
				}
			}
			angle = angle/2;
			ddElevation = ddElevation/2;
		}
		
		return false;
	}

	/**
	 * Finds the k-nearest waypoints to the given waypoint considering the
	 * problems metric
	 * 
	 * @param waypoint the waypoint in global coordinates
	 * @param num number of waypoints to return
	 * 
	 * @return list of k-nearest waypoints sorted by increasing distance
	 */
	//TODO: Correct implementation
	public List<? extends Position> findNearest(Position waypoint, int num) {
		List<Position> posiNearList = new ArrayList<Position>();
		List<Position> posiTempList = new ArrayList<Position>(waypointList);

		// sorts the list by increasing distance to waypoint
		Collections.sort(posiTempList,
				(a, b) -> super.getEnvironment().getDistance(waypoint,
						a) < super.getEnvironment().getDistance(waypoint, b)
								? -1
								: super.getEnvironment().getDistance(waypoint,
										a) == super.getEnvironment()
												.getDistance(waypoint, b) ? 0
														: 1);

		// If there are less than k neighbors on the list
		if (posiTempList.size() <= num) {
			posiNearList = posiTempList;
		} else {
			for (int k = 0; k < num; k++) {
				posiNearList.add(posiTempList.get(k));
			}
		}

		return posiNearList;
	}
}
