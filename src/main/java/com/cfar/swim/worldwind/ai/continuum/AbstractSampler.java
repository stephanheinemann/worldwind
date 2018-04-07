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

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

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

    /** the list of already sampled waypoints */
    private List<? extends Waypoint> waypointList = null;

    /**
     * Constructs a sampling based motion planner with a specified aircraft and
     * environment.
     * 
     * @param aircraft the aircraft for planning
     * @param environment the environment for planning
     */
    public AbstractSampler(Aircraft aircraft, Environment environment) {
	super(aircraft, environment);
    }

    /**
     * Gets the list of already sampled waypoints
     * 
     * @return the waypointList
     */
    public List<? extends Waypoint> getWaypointList() {
	return waypointList;
    }

    /**
     * Sets the list of waypoints previously sampled
     * 
     * @param waypointList the waypointList to set
     */
    public void setWaypointList(List<? extends Waypoint> waypointList) {
	this.waypointList = waypointList;
    }

    /**
     * Samples a position from a continuous space defined in the current
     * environment
     * 
     * @return position in global coordinates inside the environment
     */
    public Position sampleRandomPosition() {
	Position position = new Position(null, null, 0);
	
	// TODO: implement random sampler of position coordinates
	//     : based on environment borders
	
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
    public boolean checkConflict(Position waypoint) {
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
    public boolean checkConflict(Position waypoint1, Position waypoint2) {
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
    public List<Position> findNearest(Position waypoint, int num) {
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
    public Position findNearest(Position waypoint) {
	return this.findNearest(waypoint, 1).get(0);
    }
}
