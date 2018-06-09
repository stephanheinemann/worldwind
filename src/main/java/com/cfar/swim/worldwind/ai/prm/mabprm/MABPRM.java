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
package com.cfar.swim.worldwind.ai.prm.mabprm;

import java.time.ZonedDateTime;
import java.util.ArrayList;

import com.cfar.swim.worldwind.ai.astar.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * @author Henrique Ferreira
 *
 */
public class MABPRM extends BasicPRM{

	public ArrayList<Aircraft> slaveAircrafts = new ArrayList<Aircraft>();
	
	/**
	 * @return the slaveAircrafts
	 */
	public ArrayList<Aircraft> getSlaveAircrafts() {
		return slaveAircrafts;
	}
	/**
	 * @return the slaveWaypoints
	 */
	public ArrayList<ArrayList<Waypoint>> getSlaveWaypoints() {
		return slaveWaypoints;
	}
	/**
	 * @return the slaveTrajectories
	 */
	public ArrayList<Trajectory> getSlaveTrajectories() {
		return slaveTrajectories;
	}
	public void setSlaveAircrafts(ArrayList<Aircraft> slaveAircrafts) {
		this.slaveAircrafts = slaveAircrafts;
	}
	
	/** the waypoints to be visited in the trajectory of this scenario */
	public ArrayList<ArrayList<Waypoint>> slaveWaypoints = new ArrayList<ArrayList<Waypoint>>();
	
	public void setSlaveWaypoints(ArrayList<ArrayList<Waypoint>> slaveWaypoints) {
		this.slaveWaypoints = slaveWaypoints;
	}
	
	/** the planned trajectory of this scenario */
	public ArrayList<Trajectory> slaveTrajectories = new ArrayList<Trajectory>();
	
	public void setSlaveTrajectories(ArrayList<Trajectory> slaveTrajectories) {
		this.slaveTrajectories = slaveTrajectories;
	}
	/**
	 * @param aircraft
	 * @param environment
	 */
	public MABPRM(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}

	/**
	 * 
	 * @param aircraft
	 * @param environment
	 * @param maxIter
	 * @param maxNeighbors
	 * @param maxDist
	 */
	public MABPRM(Aircraft aircraft, Environment environment, int maxIter, int maxNeighbors, double maxDist) {
		super(aircraft, environment, maxIter, maxNeighbors, maxDist);
	}
	
	public void multiFindPath(ArrayList<ArrayList<Waypoint>> slaveWaypoints, Position destination, ZonedDateTime etd) {
		for(int i=0; i< this.getSlaveAircrafts().size(); i++) {
			System.out.println("creating a star");
			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getSlaveAircrafts().get(i), this.getEnvironment());
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());
			System.out.println("planning");
			this.extendsConstruction(this.getSlaveWaypoints().get(i).get(0), destination);
			System.out.println("starting wpt "+ this.getSlaveWaypoints().get(i).get(0));
			Trajectory trajectory = aStar.plan(this.getSlaveWaypoints().get(i).get(0), destination, etd);
			this.slaveTrajectories.add(trajectory);
		}
		System.out.println("revising slave plans");
		this.reviseSlavePlans(this.slaveTrajectories);
	}
}
