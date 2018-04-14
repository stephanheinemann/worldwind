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
package com.cfar.swim.worldwind.planning;

import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;
import com.cfar.swim.worldwind.ai.continuum.SampledWaypoint;
import com.cfar.swim.worldwind.ai.continuum.basicprm.BasicPRMWaypoint;
import com.cfar.swim.worldwind.geom.Box;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Polyline;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a planning roadmap that extends a planning continuum, by
 * incorporating waypoint and edge lists. Can be used for motion planning.
 * 
 * @author Henrique Ferreira
 *
 */
public class PlanningRoadmap extends PlanningContinuum implements DiscreteEnvironment {

	// TODO: extends SampledWaypoint??? waypointList defined here or in BasicPRM
	// planner?
	/** the list of sampled waypoints of this roadmap */
	private List<BasicPRMWaypoint> waypointList = new ArrayList<>();

	/** the list of edges of this roadmap */
	private List<Edge> edgeList = new ArrayList<>();

	/**
	 * Constructs a planning roadmap based on a box, a waypoint list and a edge list
	 * 
	 * @param box
	 * @param waypointList the list of waypoints
	 * @param edgeList the list of edges
	 */
	public PlanningRoadmap(Box box, List<BasicPRMWaypoint> waypointList, List<Edge> edgeList, Globe globe) {
		super(box);
		this.waypointList = waypointList;
		this.edgeList = edgeList;
		this.setGlobe(globe);
	}


	public List<BasicPRMWaypoint> getWaypointList() {
		return waypointList;
	}


	public void setWaypointList(List<BasicPRMWaypoint> waypointList) {
		this.waypointList = waypointList;
	}


	public List<Edge> getEdgeList() {
		return edgeList;
	}


	public void setEdgeList(List<Edge> edgeList) {
		this.edgeList = edgeList;
	}


	/**
	 * TODO: NOT DONE
	 * 
	 * @param position
	 * @param neighbor
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.PlanningContinuum#getLegCost(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime, com.cfar.swim.worldwind.planning.CostPolicy,
	 *      com.cfar.swim.worldwind.planning.RiskPolicy)
	 */

	@Override
	public double getLegCost(Position position, Position neighbor, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getLegCost(Environment neighbor, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getNeighbors(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public Set<Position> getNeighbors(Position position) {
		// TODO: review. Look for a better way to cast a set of sampledWaypoints to a
		// set of positions. Does it make sense to have a method working with positions
		// instead of SampledWaypoints?
		SampledWaypoint waypoint = null;
		waypoint = this.getWaypoint(position);

		Set<SampledWaypoint> sampledNeighbors = this.getNeighbors(waypoint);

		Set<Position> neighbors = new HashSet<Position>();

		for (SampledWaypoint wpt : sampledNeighbors) {
			Position pos = (Position) wpt;
			neighbors.add(pos);
		}

		return neighbors;
	}

	public Set<SampledWaypoint> getNeighbors(SampledWaypoint waypoint) {
		Set<SampledWaypoint> neighbors = new HashSet<SampledWaypoint>();

		if (null != this.getGlobe()) {
			for (Edge edge : edgeList) {
				if (waypoint.latitude==edge.getWpt1().latitude && waypoint.longitude==edge.getWpt1().longitude && waypoint.elevation==edge.getWpt1().elevation)
					neighbors.add(edge.getWpt2());
				if (waypoint.latitude==edge.getWpt2().latitude && waypoint.longitude==edge.getWpt2().longitude && waypoint.elevation==edge.getWpt2().elevation)
					neighbors.add(edge.getWpt1());
			}

		} else {
			throw new IllegalStateException("globe is not set");
		}

		return neighbors;
	}

	/**
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#isWaypoint(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isWaypoint(Position position) {
		// TODO: position.equals(waypoint) review....
		for (SampledWaypoint waypoint : waypointList) {
			if (position.equals(waypoint))
				return true;
		}
		return false;
	}

	public SampledWaypoint getWaypoint(Position position) {
		// TODO: position.equals(waypoint) review....
		for (SampledWaypoint waypoint : waypointList) {
			if (position.latitude==waypoint.latitude && position.longitude==waypoint.longitude && position.elevation==waypoint.elevation)
				return waypoint;
		}
		return null;
	}

	/**
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getAdjacentWaypoints(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public Set<Position> getAdjacentWaypoints(Position position) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * @param position
	 * @param waypoint
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#isAdjacentWaypoint(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isAdjacentWaypoint(Position position, Position waypoint) {
		// TODO: position.equals(waypoint) review...
		for (Edge edge : edgeList) {
			if (position.equals(edge.getWpt1()) && waypoint.equals(edge.getWpt2()))
				return true;
			if (position.equals(edge.getWpt2()) && waypoint.equals(edge.getWpt1()))
				return true;
		}
		return false;
	}

	/**
	 * @param costInterval
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#addCostInterval(com.cfar.swim.worldwind.planning.CostInterval)
	 */
	@Override
	public void addCostInterval(CostInterval costInterval) {
		// TODO Auto-generated method stub

	}

	/**
	 * @param costInterval
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#removeCostInterval(com.cfar.swim.worldwind.planning.CostInterval)
	 */
	@Override
	public void removeCostInterval(CostInterval costInterval) {
		// TODO Auto-generated method stub

	}

	/**
	 * @param time
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getCostIntervals(java.time.ZonedDateTime)
	 */
	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * @param start
	 * @param end
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getCostIntervals(java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime)
	 */
	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * @param start
	 * @param end
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getCost(java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime)
	 */
	@Override
	public double getCost(ZonedDateTime start, ZonedDateTime end) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * 
	 * @param origin
	 * @param destination
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.PlanningContinuum#getStepCost(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime, com.cfar.swim.worldwind.planning.CostPolicy,
	 *      com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {

		SampledWaypoint wpt1 = this.getWaypoint(origin);
		wpt1.setEto(start);
		wpt1.setCost(wpt1.calculateCost(start));
		
		SampledWaypoint wpt2 = this.getWaypoint(destination);
		wpt2.setEto(end);
		wpt2.setCost(wpt1.calculateCost(end));

		double cost = this.getStepCost(wpt1, wpt2, costPolicy, riskPolicy);

		return cost;
	}
	
	@Override
	public double getDistance(Position position1, Position position2) {
		if (null != this.getGlobe()) {
			ArrayList<Position> positions = new ArrayList<Position>();
			positions.add(position1);
			positions.add(position2);
			LengthMeasurer measurer = new LengthMeasurer(positions);
			measurer.setPathType(Polyline.LINEAR);
			measurer.setFollowTerrain(false);
			return measurer.getLength(this.getGlobe());
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the normalized distance between two positions in this planning grid.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the normalized distance between the two positions in this
	 *         planning grid
	 */
	@Override
	public double getNormalizedDistance(Position position1,
			Position position2) {
		return this.getDistance(position1, position2) / this.getDiameter();
	}

}
