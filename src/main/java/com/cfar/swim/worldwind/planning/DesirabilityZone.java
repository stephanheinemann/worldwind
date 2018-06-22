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
package com.cfar.swim.worldwind.planning;

import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.ContinuumBox;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;

/**
 * Realizes desirability zone.
 * 
 * @author Henrique Ferreira
 *
 */
public class DesirabilityZone extends ContinuumBox implements Environment {

	/** the globe of this desirability zone */
	private Globe globe = null;

	/** the current time of this desirability zone */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));

	/** the current accumulated active cost of this desirability zone */
	// TODO Review usage of active cost
	private double activeCost = 1d;

	/** the threshold cost of this desirability zone */
	private double thresholdCost = 0d;

	/** the desirability of this desirability zone */
	private double desirability = 0.5d;

	/**
	 * Constructs a desirability zone based on a geometric box and a specified
	 * desirability value.
	 * 
	 * @param box the geometric box
	 * @param desirability the desirability
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public DesirabilityZone(Box box, double desirability) {
		super(box);
		this.desirability = desirability;
		this.update();
	}

	/**
	 * Gets the threshold cost of this desirability zone.
	 * 
	 * @return the threshold cost of this desirability zone
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Sets the threshold cost of this desirability zone.
	 * 
	 * @param thresholdCost the threshold cost of this desirability zone
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double thresholdCost) {
		this.thresholdCost = thresholdCost;
		this.updateVisibility();
	}

	/**
	 * Sets the globe of this desirability zone.
	 * 
	 * @param globe the globe of this desirability zone
	 * 
	 * @see Environment#setGlobe(Globe)
	 */
	@Override
	public void setGlobe(Globe globe) {
		this.globe = globe;
	}

	/**
	 * Gets the globe of this desirability zone.
	 * 
	 * @return the globe of this desirability zone
	 * 
	 * @see Environment#getGlobe()
	 */
	@Override
	public Globe getGlobe() {
		return this.globe;
	}

	/**
	 * Gets the current time of this desirability zone.
	 * 
	 * @return the current time of this desirability zone
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the current time of this desirability zone.
	 * 
	 * @param time the current time of this desirability zone
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	// TODO: Review meaning, not clear
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}

	/**
	 * @return the desirability
	 */
	public double getDesirability() {
		return desirability;
	}

	/**
	 * @param desirability the desirability to set
	 */
	public void setDesirability(double desirability) {
		this.desirability = desirability;
	}

	/**
	 * Indicates whether or not this desirability zone contains a position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if this desirability zone contains the position, false
	 *         otherwise
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

	/**
	 * Updates this desirability zone.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
	}

	/**
	 * Updates the accumulated active cost of this desirability zone.
	 */
	protected void updateActiveCost() {
		this.activeCost = 1d;
	}

	/**
	 * Updates the visibility of this desirability zone.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}

	/**
	 * Updates the appearance of this desirability zone.
	 */
	protected void updateAppearance() {

		float r = 0, g = 0, b = 0;
		float des = (float) desirability;
		if (this.desirability <= 0.5) {
			r = 255;
			g = 510 * des;
			b = 510 * des;
		}
		if (this.desirability > 0.5) {
			r = -510 * des + 510;
			g = 255;
			b = -510 * des + 510;
		}
		float red = r / 255.0f;
		float green = g / 255.0f;
		float blue = b / 255.0f;
		float alpha = 0.5f;
		this.setColor(red, green, blue, alpha);
	}

	/**
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#isWaypoint(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isWaypoint(Position position) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getAdjacentWaypoints(gov.nasa.worldwind.geom.Position)
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
	 * @see com.cfar.swim.worldwind.planning.Environment#isAdjacentWaypoint(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isAdjacentWaypoint(Position position, Position waypoint) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getCenterPosition()
	 */
	@Override
	public Position getCenterPosition() {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getNeighbors()
	 */
	@Override
	public Set<? extends Environment> getNeighbors() {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * @param neighbor
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#areNeighbors(com.cfar.swim.worldwind.planning.Environment)
	 */
	@Override
	public boolean areNeighbors(Environment neighbor) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getNeighbors(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public Set<Position> getNeighbors(Position position) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * @param position
	 * @param neighbor
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#areNeighbors(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean areNeighbors(Position position, Position neighbor) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#isRefined()
	 */
	@Override
	public boolean isRefined() {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getRefinements()
	 */
	@Override
	public Set<? extends Environment> getRefinements() {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * @param density
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#refine(int)
	 */
	@Override
	public void refine(int density) {
		// TODO Auto-generated method stub

	}

	/**
	 * 
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#coarsen()
	 */
	@Override
	public void coarsen() {
		// TODO Auto-generated method stub

	}

	/**
	 * @param obstacle
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#embed(com.cfar.swim.worldwind.render.Obstacle)
	 */
	@Override
	public boolean embed(Obstacle obstacle) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @param obstacle
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#unembed(com.cfar.swim.worldwind.render.Obstacle)
	 */
	@Override
	public boolean unembed(Obstacle obstacle) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * 
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#unembedAll()
	 */
	@Override
	public void unembedAll() {
		// TODO Auto-generated method stub

	}

	/**
	 * @param obstacle
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#isEmbedded(com.cfar.swim.worldwind.render.Obstacle)
	 */
	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @param obstacle
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#refresh(com.cfar.swim.worldwind.render.Obstacle)
	 */
	@Override
	public void refresh(Obstacle obstacle) {
		// TODO Auto-generated method stub

	}

	/**
	 * @param position1
	 * @param position2
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getDistance(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public double getDistance(Position position1, Position position2) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @param position1
	 * @param position2
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getNormalizedDistance(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @param costInterval
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#addCostInterval(com.cfar.swim.worldwind.planning.CostInterval)
	 */
	@Override
	public void addCostInterval(CostInterval costInterval) {
		// TODO Auto-generated method stub

	}

	/**
	 * @param costInterval
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#removeCostInterval(com.cfar.swim.worldwind.planning.CostInterval)
	 */
	@Override
	public void removeCostInterval(CostInterval costInterval) {
		// TODO Auto-generated method stub

	}

	/**
	 * @param time
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getCostIntervals(java.time.ZonedDateTime)
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
	 * @see com.cfar.swim.worldwind.planning.Environment#getCostIntervals(java.time.ZonedDateTime,
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
	 * @see com.cfar.swim.worldwind.planning.Environment#getCost(java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime)
	 */
	@Override
	public double getCost(ZonedDateTime start, ZonedDateTime end) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @param origin
	 * @param destination
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getStepCost(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime, com.cfar.swim.worldwind.planning.CostPolicy,
	 *      com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	public double getDesirabilityCost(Edge edge) {
		if (this.intersects(edge.getLine())) {
			return this.desirability;
		}
		return 0.5;
	}

	/**
	 * @param origin
	 * @param destination
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getLegCost(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime, com.cfar.swim.worldwind.planning.CostPolicy,
	 *      com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getLegCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @param destination
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getLegCost(com.cfar.swim.worldwind.planning.Environment,
	 *      java.time.ZonedDateTime, java.time.ZonedDateTime,
	 *      com.cfar.swim.worldwind.planning.CostPolicy,
	 *      com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getLegCost(Environment destination, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}
}
