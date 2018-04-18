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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.ai.continuum.SampledWaypoint;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.ContinuumBox;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.TerrainCylinder;
import com.cfar.swim.worldwind.render.TerrainObstacle;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Polyline;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a planning continuum that implements an environment, can be used for
 * sampled based motion planning.
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public class PlanningContinuum extends ContinuumBox implements Environment {

	/** the globe of this planning continuum */
	private Globe globe = null;

	/** the current time of this planning continuum */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));

	/** the obstacles embedded into this planning continuum*/
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();
	
	/** the terrain obstacles embedded into this planning continuum*/
	private HashSet<TerrainObstacle> terrainObstacles = new HashSet<TerrainObstacle>();

	/** the current accumulated active cost of this planning continuum */
	// TODO Review usage of active cost
	private double activeCost = 1d;

	/** the threshold cost of this planning continuum */
	private double thresholdCost = 0d;
	
	/** the resolution of this planning continuum */
	private double resolution = 1d;

	/**
	 * Constructs a planning continuum based on a geometric box.
	 * 
	 * @param box the geometric box
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public PlanningContinuum(Box box) {
		super(box);
		this.update();
	}
	
	/**
	 * Constructs a planning continuum based on a geometric box.
	 * 
	 * @param box the geometric box
	 * @param resolution the resolution of this planning continuum
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public PlanningContinuum(Box box, double resolution) {
		super(box);
		this.resolution = resolution;
		this.update();
	}

	/**
	 * Gets the current time of this planning continuum.
	 * 
	 * @return the current time of this planning continuum
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the current time of this planning continuum.
	 * 
	 * @param time the current time of this planning continuum
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
	 * Gets the threshold cost of this planning continuum.
	 * 
	 * @return the threshold cost of this planning continuum
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Sets the threshold cost of this planning continuum.
	 * 
	 * @param thresholdCost the threshold cost of this planning continuum
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double thresholdCost) {
		this.thresholdCost = thresholdCost;
		this.updateVisibility();
	}

	/**
	 * Sets the globe of this planning continuum.
	 * 
	 * @param globe the globe of this planning continuum
	 * 
	 * @see Environment#setGlobe(Globe)
	 */
	@Override
	public void setGlobe(Globe globe) {
		this.globe = globe;
	}

	/**
	 * Gets the globe of this planning continuum.
	 * 
	 * @return the globe of this planning continuum
	 * 
	 * @see Environment#getGlobe()
	 */
	@Override
	public Globe getGlobe() {
		return this.globe;
	}

	/**
	 * Gets the obstacles of this planning continuum.
	 * 
	 * @return the obstacles
	 */
	public HashSet<Obstacle> getObstacles() {
		return obstacles;
	}

	/**
	 * Sets the obstacles of this planning continuum.
	 * 
	 * @param obstacles the obstacles to set
	 */
	public void setObstacles(HashSet<Obstacle> obstacles) {
		this.obstacles = obstacles;
	}

	/**
	 * Gets the resolution of this planning continuum
	 * 
	 * @return the resolution of this planning continuum
	 */
	public double getResolution() {
		return resolution;
	}
	
	/**
	 * Sets the resolution of this planning continuum
	 * 
	 * @param resolution the resolution to set
	 */
	public void setResolution(double resolution) {
		this.resolution = resolution;
	}

	/**
	 * Gets the terrain obstacles of this planning continuum.
	 * 
	 * @return the terrain obstacles
	 */
	public HashSet<TerrainObstacle> getTerrainObstacles() {
		return terrainObstacles;
	}

	/**
	 * Sets the terrain obstacles of this planning continuum.
	 * 
	 * @param terrainObstacles the terrain obstacles to set
	 */
	public void setTerrainObstacles(HashSet<TerrainObstacle> terrainObstacles) {
		this.terrainObstacles = terrainObstacles;
	}

	/**
	 * Indicates whether or not this planning continuum contains a position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if this planning continuum contains the position, false otherwise
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
	 * Gets the center position in globe coordinates of this planning continuum.
	 * 
	 * @return the center position in globe coordinates of this planning continuum
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

	// TODO What are neighboring environments?
	@Override
	public Set<? extends Environment> getNeighbors() {
		return null;
	}

	@Override
	public boolean areNeighbors(Environment neighbor) {
		return false;
	}

	/**
	 * Gets the distance between two positions in this planning continuum.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the distance between the two positions in this planning continuum
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
	 * Gets the normalized distance between two positions in this planning continuum.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the normalized distance between the two positions in this planning
	 *         continuum
	 */
	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		return this.getDistance(position1, position2) / this.getDiameter();
	}

	/**
	 * Gets the step cost from an origin to a destination position within this
	 * planning continuum between a start and an end time given a cost policy and risk
	 * policy.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the step cost from the origin to the destination position
	 */
	// TODO: We implement the same method defined in the interface but need to
	// receive an extension of a position (SmapledWaypoint)
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		return 0;
	}

	public double getStepCost(SampledWaypoint origin, SampledWaypoint destination, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {

		double stepCost = 0d;
		List<SampledWaypoint> waypoints = new ArrayList<SampledWaypoint>();

		waypoints.add(origin);
		// compute participating cells
		// TODO: discretize edge in multiple waypoints
		waypoints.add(destination);

		List<Double> costs = new ArrayList<Double>();

		// compute initial distance cost
		double distance, cost;
		SampledWaypoint waypoint1 = waypoints.get(0);
		SampledWaypoint waypoint2;

		// compute cost of each adjacent waypoint
		for (int i = 0; i < waypoints.size() - 1; i++) {
			waypoint2 = waypoints.get(i + 1);
			distance = this.getDistance(waypoint1, waypoint2);
			cost = (waypoint1.getCost() + waypoint2.getCost()) / 2;

			// boost cost if local risk is not acceptable
			if (riskPolicy.satisfies(cost - 1)) {
				costs.add(distance * cost);
			} else {
				costs.add(Double.POSITIVE_INFINITY);
			}
			waypoint1 = waypoint2;
		}
			
		// apply cost policy for final cost
		switch (costPolicy) {
		case MINIMUM:
			stepCost = costs.stream().mapToDouble(Double::doubleValue).min().getAsDouble();
			break;
		case MAXIMUM:
			stepCost = costs.stream().mapToDouble(Double::doubleValue).max().getAsDouble();
			break;
		case AVERAGE:
			stepCost = costs.stream().mapToDouble(Double::doubleValue).average().getAsDouble();
			break;
		}

		return stepCost;
	}

	@Override
	public double getLegCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getLegCost(Environment destination, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * Embeds an obstacle into this planning continuum.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
	 */
	public boolean embed(ObstacleCylinder obstacle) {
		boolean embedded = false;

		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.obstacles.add(obstacle);

				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return embedded;
	}
	
	/**
	 * Embeds an obstacle into this planning continuum.
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
				this.obstacles.add(obstacle);

				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return embedded;
	}

	/**
	 * Unembeds an obstacle from this planning continuum.
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

			this.obstacles.remove(obstacle);

			unembedded = true;
		}

		return unembedded;
	}

	/**
	 * Updates this planning continuum for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @see Environment#refresh(Obstacle)
	 */
	@Override
	public void refresh(Obstacle obstacle) {
		if (this.obstacles.contains(obstacle)) {
			this.update();
		}
	}

	/**
	 * Unembeds all obstacles from this planning continuum.
	 * 
	 * @see Environment#unembedAll()
	 */
	@Override
	public void unembedAll() {
		this.obstacles.clear();
	}

	/**
	 * Indicates whether or not an obstacle is embedded in this planning continuum.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this planning continuum, false
	 *         otherwise
	 * 
	 * @see Environment#isEmbedded(Obstacle)
	 */
	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		return this.obstacles.contains(obstacle);
	}

	/**
	 * Embeds an obstacle into this planning continuum.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
	 */
	public boolean embed(TerrainCylinder obstacle) {
		boolean embedded = false;

		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.terrainObstacles.add(obstacle);

				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return embedded;
	}
	
	/**
	 * Embeds an obstacle into this planning continuum.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
	 */
	public boolean embed(TerrainObstacle obstacle) {
		boolean embedded = false;

		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.terrainObstacles.add(obstacle);

				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return embedded;
	}

	/**
	 * Unembeds an obstacle from this planning continuum.
	 * 
	 * @param obstacle the obstacle to be unembedded
	 * 
	 * @return true if the obstacle has been unembedded, false otherwise
	 * 
	 * @see Environment#unembed(Obstacle)
	 */
	public boolean unembed(TerrainObstacle obstacle) {
		boolean unembedded = false;

		if (this.isEmbedded(obstacle)) {

			this.terrainObstacles.remove(obstacle);

			unembedded = true;
		}

		return unembedded;
	}

	/**
	 * Updates this planning continuum for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @see Environment#refresh(Obstacle)
	 */
	public void refresh(TerrainObstacle obstacle) {
		if (this.terrainObstacles.contains(obstacle)) {
			this.update();
		}
	}

	/**
	 * Unembeds all obstacles from this planning continuum.
	 * 
	 * @see Environment#unembedAll()
	 */
	public void unembedTerrainAll() {
		this.terrainObstacles.clear();
	}

	/**
	 * Indicates whether or not an obstacle is embedded in this planning continuum.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this planning continuum, false
	 *         otherwise
	 * 
	 * @see Environment#isEmbedded(Obstacle)
	 */
	public boolean isEmbedded(TerrainObstacle obstacle) {
		return this.terrainObstacles.contains(obstacle);
	}
	
	/**
	 * Indicates whether or not this planning grid is refined, that is, has
	 * children.
	 * 
	 * @return true if this planning grid is refined, false otherwise
	 * 
	 */
	public boolean isRefined() {
		//TODO: review maximum resolution
		return this.getResolution()<=0.01;
	}

//	/**
//	 * Gets the refinements, that is, children of this planning grid.
//	 * 
//	 * @return the refinements of this planning grid
//	 * 
//	 */
//	public Set<? extends PlanningGrid> getRefinements() {
//		return this.getChildren();
//	}

	/**
	 * Refines, that is, adds children with a specified density to this planning
	 * grid.
	 * 
	 * @param density the refinement density
	 * 
	 */
	public void refine(int percentage) {
		this.setResolution(this.getResolution()*(1-percentage/100d));
	}

	/**
	 * Coarsens, that is, removes the children of this planning grid.
	 *
	 */
	public void coarsen() {
		this.setResolution(this.getResolution()*1.5);
	}
	
	/**
	 * Updates this planning continuum.
	 */
	protected void update() {
		this.updateAppearance();
		this.updateVisibility();
	}

	/**
	 * Updates the visibility of this planning continuum.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}

	/**
	 * Updates the appearance of this planning continuum.
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
	 * Gets the interval tree that is defined for a specified position.
	 * 
	 * @param position the position to be checked
	 * 
	 * @return the interval tree with all cost intervals
	 */
	public IntervalTree<ChronoZonedDateTime<?>> getIntervalTree(Position position) {
		IntervalTree<ChronoZonedDateTime<?>> intervalTree = new IntervalTree<ChronoZonedDateTime<?>>(
				CostInterval.comparator);

		if (null != this.globe) {
			for (Obstacle obstacle : obstacles) {
				if (this.intersects(obstacle.getExtent(this.globe))) {
					intervalTree.add(obstacle.getCostInterval());
				}
			}
		}

		return intervalTree;
	}
	
	public IntervalTree<ChronoZonedDateTime<?>> embedIntervalTree(Position position) {
		IntervalTree<ChronoZonedDateTime<?>> intervalTree = new IntervalTree<ChronoZonedDateTime<?>>(
				CostInterval.comparator);
		Box box = this.createBoundingBox(position);
		
		for (Obstacle obstacle : this.getObstacles()) {
			if(obstacle.getExtent(this.getGlobe()).intersects(box.getFrustum())) {
				intervalTree.add(obstacle.getCostInterval());
			}
		}
		return intervalTree;
	}
	
	public Box createBoundingBox(Position position) {
		Vec4 point = this.getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();
		
		//TODO: create box according to aircraft dimensions
		double halfDistance = 0.1d;
		
		corners.add(point.add3(-halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, +halfDistance, +halfDistance));
		
		return new Box(gov.nasa.worldwind.geom.Box.computeBoundingBox(corners));
	}

}
