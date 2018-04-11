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

import java.awt.Color;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.ai.continuum.basicprm.BasicPRMWaypoint;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.ContinuumBox;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.geom.CubicGrid;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Polyline;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

// TODO: a planning roadmap could be fed by AIXM data to establish valid VFR/IFR routes
// TODO: there could be a geometric roadmap base class (similar to planning grid)
public class PlanningRoadmap extends ContinuumBox implements DiscreteEnvironment {

	/** the globe of this planning grid */
	private Globe globe = null;
	
	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(CostInterval.comparator);
	
	/** the current time of this planning grid */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the current accumulated active cost of this planning grid */
	private double activeCost = 1d;
	
	/** the threshold cost of this planning grid */
	private double thresholdCost = 0d;
	
	/** the obstacles embedded into this planning grid */
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();
	
	private List<? extends Waypoint> waypointList = null;
	
	private List<Edge> edgeList = null;
	
	/**
	 * Constructs a planning grid based on a geometric cube without any
	 * children.
	 * 
	 * @param cube the geometric cube
	 * 
	 * @see CubicGrid#CubicGrid(Cube)
	 */
	public PlanningRoadmap(Box box, List<? extends Waypoint> waypointList, List<Edge> edgeList) {
		super(box);
		this.waypointList=waypointList;
		this.edgeList=edgeList;
		this.update();
	}
	
	/**
	 * Gets the current time of this planning grid.
	 * 
	 * @return the current time of this planning grid
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}
	
	/**
	 * Sets the current time of this planning grid.
	 * 
	 * @param time the current time of this planning grid
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}

	/**
	 * Gets the threshold cost of this planning grid.
	 * 
	 * @return the threshold cost of this planning grid
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}
	
	/**
	 * Sets the threshold cost of this planning grid.
	 * 
	 * @param thresholdCost the threshold cost of this planning grid
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double thresholdCost) {
		this.thresholdCost = thresholdCost;
		this.updateVisibility();
	}

	/**
	 * Sets the globe of this planning grid.
	 * 
	 * @param globe the globe of this planning grid
	 * 
	 * @see Environment#setGlobe(Globe)
	 */
	@Override
	public void setGlobe(Globe globe) {
		this.globe = globe;
	}
	
	/**
	 * Gets the globe of this planning grid.
	 * 
	 * @return the globe of this planning grid
	 * 
	 * @see Environment#getGlobe()
	 */
	@Override
	public Globe getGlobe() {
		return this.globe;
	}

	/**
	 * Gets the center position in globe coordinates of this planning grid.
	 * 
	 * @return the center position in globe coordinates of this planning grid
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#getCenter()
	 */
	@Override
	public Position getCenterPosition() {
		Position centerPosition = null;
		
		if (null != this.globe) {
			centerPosition = this.globe.computePositionFromPoint(this.getCenter());
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return centerPosition;
	}

	public List<? extends Waypoint> getWaypointList() {
		return waypointList;
	}

	public void setWaypointList(List<? extends Waypoint> waypointList) {
		this.waypointList = waypointList;
	}

	public List<Edge> getEdgeList() {
		return edgeList;
	}

	public void setEdgeList(List<Edge> edgeList) {
		this.edgeList = edgeList;
	}
	
	/**
	 * NOT DONE
	 * @return
	
	 * @see com.cfar.swim.worldwind.planning.Environment#getNeighbors()
	 */
	@Override
	public Set<? extends Environment> getNeighbors() {
		// TODO Auto-generated method stub
		return null;
	}
	
	/**
	 * NOT DONE
	 * @param neighbor
	 * @return
	
	 * @see com.cfar.swim.worldwind.planning.Environment#areNeighbors(com.cfar.swim.worldwind.planning.Environment)
	 */
	@Override
	public boolean areNeighbors(Environment neighbor) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * Gets the distance between two positions in this planning grid.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the distance between the two positions in this planning grid
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Environment#getDistance(Position, Position)
	 */
	@Override
	public double getDistance(Position position1, Position position2) {
		if (null != this.globe) {
			ArrayList<Position> positions = new ArrayList<Position>();
			positions.add(position1);
			positions.add(position2);
			LengthMeasurer measurer = new LengthMeasurer(positions);
			measurer.setPathType(Polyline.LINEAR);
			measurer.setFollowTerrain(false);
			return measurer.getLength(this.globe);
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
	public double getNormalizedDistance(Position position1, Position position2) {
		return this.getDistance(position1, position2) / this.getDiameter();
	}

	@Override
	public void addCostInterval(CostInterval costInterval) {
		// TODO Auto-generated method stub

	}

	@Override
	public void removeCostInterval(CostInterval costInterval) {
		// TODO Auto-generated method stub

	}

	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double getCost(ZonedDateTime start, ZonedDateTime end) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getLegCost(
			Position position, Position neighbor, 
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getLegCost(
			Environment neighbor,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * Indicates whether or not this planning grid contains a position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if this planning grid contains the position,
	 *         false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Environment#contains(Position)
	 * @see Box#contains(Vec4)
	 */
	@Override
	public boolean contains(Position position) {
		if (null != this.globe) {
			return super.contains(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	@Override
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	public void coarsen() {
	}

	/**
	 * Embeds an obstacle into this planning grid.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
	 */
	@Override
	public boolean embed(Obstacle obstacle) {
		boolean embedded = false;
		
		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.addCostInterval(obstacle.getCostInterval());
				this.obstacles.add(obstacle);
				
				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return embedded;
	}

	/**
	 * Unembeds an obstacle from this planning grid.
	 * 
	 * @param obstacle the obstacle to be unembedded
	 * 
	 * @return true if the obstacle has been unembedded, false otherwise
	 * 
	 * @see Environment#unembed(Obstacle)
	 */
	@Override
	public boolean unembed(Obstacle obstacle) {
		boolean unembedded = false;
		
		if (this.isEmbedded(obstacle)) {
			this.removeCostInterval(obstacle.getCostInterval());
			this.obstacles.remove(obstacle);
			
			unembedded = true;
		}
		
		return unembedded;
	}

	/**
	 * Updates this planning grid for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @see Environment#refresh(Obstacle)
	 */
	@Override
	public void refresh(Obstacle obstacle) {
		if (this.obstacles.contains(obstacle))
			this.update();
	}

	/**
	 * Indicates whether or not an obstacle is embedded in this planning grid.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this planning grid,
	 *         false otherwise
	 * 
	 * @see Environment#isEmbedded(Obstacle)
	 */
	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		return this.obstacles.contains(obstacle);
	}

	/**
	 * Unembeds all obstacles from this planning grid.
	 * 
	 * @see Environment#unembedAll()
	 */
	@Override
	public void unembedAll() {
		Iterator<Obstacle> obstaclesIterator = this.obstacles.iterator();
		while (obstaclesIterator.hasNext()) {
			Obstacle obstacle = obstaclesIterator.next();
			this.removeCostInterval(obstacle.getCostInterval());
			obstaclesIterator.remove();
			
		}
	}

	/**
	 * @param position
	 * @return
	
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#isWaypoint(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isWaypoint(Position position) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @param position
	 * @return
	
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
	
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#isAdjacentWaypoint(gov.nasa.worldwind.geom.Position, gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isAdjacentWaypoint(Position position, Position waypoint) {
		// TODO Auto-generated method stub
		return false;
	}
	
	/**
	 * Updates this planning grid.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
	}
	
	/**
	 * Updates the accumulated active cost of this planning grid.
	 */
	protected void updateActiveCost() {
		this.activeCost = this.getCost(this.time);
	}
	
	/**
	 * Updates the visibility of this planning grid.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}
	
	/**
	 * Updates the appearance of this planning grid.
	 */
	protected void updateAppearance() {
		Color activeColor = ObstacleColor.getColor(activeCost);
		float red = activeColor.getRed() / 255.0f;
		float green = activeColor.getGreen() / 255.0f;
		float blue = activeColor.getBlue() / 255.0f;
		float alpha = activeColor.getAlpha() / 255.0f;
		this.setColor(red, green, blue, alpha);
	}
	
	/**
	 * Gets the accumulated cost of this planning grid at specified time
	 * instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return the accumulated cost of this planning grid at the specified
	 *         time instant
	 */
	public double getCost(ZonedDateTime time) {
		return this.getCost(time, time);
	}
	
	

}
