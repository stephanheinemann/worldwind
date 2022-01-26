/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.environments;

import java.awt.Color;
import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.geom.CubicGrid;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.WeightedCostInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;

import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.terrain.HighResolutionTerrain;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a planning grid that can encode spatial and temporal costs, and can
 * be used for motion planning.
 * 
 * @author Stephan Heinemann
 *
 */
public class PlanningGrid extends CubicGrid
implements DynamicHierarchicalEnvironment, MultiResolutionEnvironment {
	
	/** the base cost of this planning grid */
	private static final double BASE_COST = 1d;
	
	/** the globe of this planning grid */
	private Globe globe = null;
	
	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(CostInterval.comparator);
	
	/** the current time of this planning grid */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the current accumulated active cost of this planning grid */
	private double activeCost = PlanningGrid.BASE_COST;
	
	/** the threshold cost of this planning grid */
	private double thresholdCost = 0d;
	
	/** the obstacles embedded into this planning grid */
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();
	
	/** the affected children of obstacle embeddings */
	private HashMap<Obstacle, List<PlanningGrid>> affectedChildren = new HashMap<Obstacle, List<PlanningGrid>>();
	
	/** the structural change listeners of this planning grid */
	private Set<StructuralChangeListener> listeners = new HashSet<StructuralChangeListener>();
	
	/**
	 * Constructs a planning grid based on a geometric cube without any
	 * children.
	 * 
	 * @param cube the geometric cube
	 * 
	 * @see CubicGrid#CubicGrid(Cube)
	 */
	public PlanningGrid(Cube cube) {
		super(cube);
		this.update();
	}
	
	/**
	 * Constructs a planning grid from a geometric cube representing a
	 * reference child.
	 * 
	 * @param refChild the geometric cube representing the reference child
	 * @param rCells the number of cubic children along the <code>R</code> axis
	 * @param sCells the number of cubic children along the <code>S</code> axis
	 * @param tCells the number of cubic children along the <code>T</code> axis
	 *
	 * @see CubicGrid#CubicGrid(Cube, int, int, int)
	 */
	public PlanningGrid(Cube refChild, int rCells, int sCells, int tCells) {
		super(refChild, rCells, sCells, tCells);
		this.refresh();
	}
	
	/**
	 * Constructs a new planning grid from three plane normals and six
	 * distances for each of the six faces of a geometric box without any
	 * children.
	 * 
	 * This factory method is used during child construction and supposed to be
	 * overridden by specializing classes.
	 * 
	 * @see CubicGrid#newInstance(Vec4[], double, double, double, double, double, double)
	 */
	@Override
	protected PlanningGrid newInstance(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		
		Box b = new Box(axes, rMin, rMax, sMin, sMax, tMin, tMax);
		return new PlanningGrid(new Cube(b.getOrigin(), axes, b.getRLength()));
	}
	
	/**
	 * Determines whether or not this planning grid has a globe.
	 * 
	 * @return true if this planning grid has a globe, false otherwise
	 * 
	 * @see Environment#hasGlobe()
	 */
	@Override
	public boolean hasGlobe() {
		return (null != this.globe);
	}
	
	/**
	 * Determines whether or not a position is inside the globe of this
	 * planning grid.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is inside the globe of this planning grid,
	 *         false otherwise
	 * 
	 * @throws IllegalStateException if this planning grid has no globe
	 * 
	 * @see Environment#isInsideGlobe(Position)
	 */
	@Override
	public boolean isInsideGlobe(Position position) {
		if (this.hasGlobe()) {
			double elevation = this.getGlobe()
					.getElevation(position.latitude, position.longitude);
			return position.elevation < elevation;
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Determines whether or not this planning grid contains a position.
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
		if (this.hasGlobe()) {
			return super.contains(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Gets the eight corner positions in globe coordinates of this
	 * planning grid.
	 *  
	 * @return the eight corner positions in globe coordinates of this
	 *         planning grid
	 * 
	 * @throws IllegalStateException if the globe is not set
	 *         
	 * @see Box#getCorners()
	 */
	public Position[] getCornerPositions() {
		Position[] cornerPositions = null;
		
		if (this.hasGlobe()) {
			cornerPositions = new Position[8];
			Vec4[] corners = this.getCorners();
			for (int index = 0; index < 8; index++) {
				cornerPositions[index] = this.globe.computePositionFromPoint(corners[index]);
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return cornerPositions;
	}
	
	/**
	 * Determines whether or not a position in globe coordinates is a corner of
	 * this planning grid considering numerical inaccuracies.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is a corner of this planning grid
	 *         considering numerical inaccuracies, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#isCorner(Vec4)
	 */
	public boolean isCorner(Position position) {
		boolean isCorner = false;
		
		if (this.hasGlobe()) {
			isCorner = this.isCorner(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return isCorner;
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
		
		if (this.hasGlobe()) {
			centerPosition = this.globe.computePositionFromPoint(this.getCenter());
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return centerPosition;
	}
	
	/**
	 * Determines whether or not a position in globe coordinates is the center
	 * of this planning grid considering numerical inaccuracies.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is the center of this planning grid
	 *         considering numerical inaccuracies, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#isCorner(Vec4)
	 */
	public boolean isCenter(Position position) {
		boolean isCenter = false;
		
		if (this.hasGlobe()) {
			isCenter = this.isCenter(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return isCenter;
	}
	
	/**
	 * Gets the neighbor corner positions of a specified corner position of
	 * this planning grid.
	 * 
	 * @param position the specified corner position
	 * 
	 * @return the neighbor corner positions of a the specified corner position
	 *         if a corner position, null otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#getNeighborCorners(Vec4)
	 */
	public Position[] getNeighborCorners(Position position) {
		Position[] neighborCorners = null;
		
		if (this.hasGlobe()) {
			neighborCorners = new Position[3];
			Vec4[] corners = this.getNeighborCorners(globe.computePointFromPosition(position));
			
			if (null != corners) {
				neighborCorners[0] = this.globe.computePositionFromPoint(corners[0]);
				neighborCorners[1] = this.globe.computePositionFromPoint(corners[1]);
				neighborCorners[2] = this.globe.computePositionFromPoint(corners[2]);
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return neighborCorners;
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
		for (PlanningGrid grid : this.getAll()) {
			grid.globe = globe;
		}
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
	 * Adds a cost interval to this planning grid.
	 * 
	 * @param costInterval the cost interval to be added
	 * 
	 * @see Environment#addCostInterval(CostInterval)
	 */
	@Override
	public void addCostInterval(CostInterval costInterval) {
		this.costIntervals.add(costInterval);
		this.update();
		// TODO: Should costs be automatically propagated to the affected child cells?
		// TODO: Should added children costs be propagated to parents?
		// TODO: What happens if children are added and removed?
		// TODO: Shall parents aggregate all costs?!
	}
	
	/**
	 * Removes a cost interval from this planning grid.
	 * 
	 * @param costInterval the cost interval to be removed
	 * 
	 * @see Environment#removeCostInterval(CostInterval)
	 */
	@Override
	public void removeCostInterval(CostInterval costInterval) {
		this.costIntervals.remove(costInterval);
		this.update();
	}
	
	/**
	 * Gets all cost intervals that are active at a specified time instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return all cost intervals that are active at the specified time instant
	 * 
	 * @see Environment#getCostIntervals(ZonedDateTime)
	 */
	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		return this.costIntervals.searchInterval(new CostInterval(null, time));
	}
	
	/**
	 * Gets all cost intervals that are active during a specified time interval.
	 * 
	 * @param start the start time of the time interval
	 * @param end the end time of the time interval
	 * 
	 * @return all cost intervals that are active during the specified time
	 *         interval
	 * 
	 * @see Environment#getCostIntervals(ZonedDateTime, ZonedDateTime)
	 */
	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		return this.costIntervals.searchInterval(new CostInterval(null, start, end));
	}
	
	/**
	 * Gets the interval limits of all sub-cost intervals contained in this
	 * planning grid within a specified time span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the interval limits of all sub-cost intervals contained in this
	 *         planning grid within the specified time span
	 */
	protected TreeSet<ZonedDateTime> getSubCostIntervalLimits(
			ZonedDateTime start, ZonedDateTime end) {
		// cost sub-interval time limits
		TreeSet<ZonedDateTime> subCostIntervalLimits =
				new TreeSet<ZonedDateTime>();
		subCostIntervalLimits.add(start);
		subCostIntervalLimits.add(end);
		
		// add cost sub-interval time limits within enclosing interval
		for (Interval<ChronoZonedDateTime<?>> interval :
			this.getCostIntervals(start, end)) {
			
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;
				ZonedDateTime lower = costInterval.getLower();
				ZonedDateTime upper = costInterval.getUpper();
				if (lower.isAfter(start))
					subCostIntervalLimits.add(costInterval.getLower());
				if (upper.isBefore(end))
					subCostIntervalLimits.add(costInterval.getUpper());
			}
		}
		
		return subCostIntervalLimits;
	}
	
	/**
	 * Gets all sub-cost intervals contained in this planning grid within a
	 * specified time span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the sub-cost intervals contained in this planning grid within
	 *         the specified time span
	 */
	protected Set<CostInterval> getSubCostIntervals(
			ZonedDateTime start, ZonedDateTime end) {
		Set<CostInterval> subCostIntervals = new HashSet<CostInterval>();
		
		// obtain sub-cost interval limits
		Collection<ZonedDateTime> subCostIntervalLimits;
		if (start.equals(end)) {
			subCostIntervalLimits = new ArrayList<ZonedDateTime>(2);
			subCostIntervalLimits.add(start);
			subCostIntervalLimits.add(end);
		} else {
			subCostIntervalLimits = this.getSubCostIntervalLimits(start, end);
		}
		
		Iterator<ZonedDateTime> subCostIntervalLimitsIterator =
				subCostIntervalLimits.iterator();
		ZonedDateTime subCostIntervalStart = subCostIntervalLimitsIterator.next();
		ZonedDateTime subCostIntervalEnd;

		// calculate sub-cost intervals
		while (subCostIntervalLimitsIterator.hasNext()) {
			subCostIntervalEnd = subCostIntervalLimitsIterator.next();
			double subIntervalCost = this.getBaseCost();
			Set<String> costIntervalIds = new HashSet<String>();
			
			for (Interval<ChronoZonedDateTime<?>> interval :
				this.getCostIntervals(start, end)) {
				
				if (interval instanceof CostInterval) {
					CostInterval costInterval = (CostInterval) interval;
					ZonedDateTime lower = costInterval.getLower();
					ZonedDateTime upper = costInterval.getUpper();
					
					// only consider fully contained intervals
					if (!lower.isAfter(subCostIntervalStart)
							&& !upper.isBefore(subCostIntervalEnd)) {
						
						// only add different overlapping cost intervals
						if (!costIntervalIds.contains(costInterval.getId())) {
							costIntervalIds.add(costInterval.getId());
							
							// TODO: implement a proper weighted cost calculation normalized from 0 to 100
							// TODO: the weight is affected by severity (reporting method) and currency (reporting time)
							
							if ((interval instanceof WeightedCostInterval)) {
								subIntervalCost += ((WeightedCostInterval)
										interval).getWeightedCost();
							} else {
								subIntervalCost += costInterval.getCost();
							}
						}
					}
				}
			}
			
			subCostIntervals.add(new CostInterval(
					subCostIntervalStart + "-" + subCostIntervalEnd,
					subCostIntervalStart, subCostIntervalEnd, subIntervalCost));
			subCostIntervalStart = subCostIntervalEnd;
		}
		
		return subCostIntervals;
	}
	
	/**
	 * Gets the base cost of a normalized step in this planning grid.
	 * 
	 * @return the base cost of a normalized step in this planning grid
	 * 
	 * @see Environment#getBaseCost()
	 */
	@Override
	public double getBaseCost() {
		return PlanningGrid.BASE_COST;
	}
	
	/**
	 * Gets the accumulated cost of this planning grid at specified time
	 * instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return the accumulated cost of this planning grid at the specified
	 *         time instant
	 * 
	 * @see Environment#getCost(ZonedDateTime)
	 */
	@Override
	public double getCost(ZonedDateTime time) {
		return this.getCost(time, time);
	}
	
	/**
	 * Gets the accumulated cost of this planning grid within a specified time
	 * span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the accumulated cost of this planning grid within the specified
	 *         time span
	 * 
	 * @see Environment#getCost(ZonedDateTime, ZonedDateTime)
	 */
	@Override
	public double getCost(ZonedDateTime start, ZonedDateTime end) {
		double cost = 0d;
		
		// obtain sub-cost intervals
		Set<CostInterval> subCostIntervals =
				this.getSubCostIntervals(start, end);
		
		// assume maximum cost
		cost = subCostIntervals.stream()
				.map(c -> c.getCost()).mapToDouble(Double::doubleValue)
				.max().getAsDouble();
		
		return cost;
	}
	
	/**
	 * Gets the accumulated cost of this planning grid within a specified time
	 * span applying an operational cost and risk policy.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * @param costPolicy the operational cost policy to be applied
	 * @param riskPolicy the operational risk policy to be applied
	 * 
	 * @return the accumulated cost of this planning grid within the specified
	 *         time span after applying the operational cost and risk policy
	 */
	@Override
	public double getCost(ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		double cost = 0d;
		
		// obtain sub-cost intervals
		Set<CostInterval> subCostIntervals =
				this.getSubCostIntervals(start, end);
		
		// adhere to operational risk policy
		subCostIntervals.stream()
			.forEach(c -> {
				if (!riskPolicy.satisfies(c.getCost() - this.getBaseCost()))
					c.setCost(Double.POSITIVE_INFINITY);
			});
		
		// apply operational cost policy
		switch (costPolicy) {
		case MINIMUM:
			cost = subCostIntervals.stream()
				.map(c -> c.getCost()).mapToDouble(Double::doubleValue)
				.min().getAsDouble();
			break;
		case MAXIMUM:
			cost = subCostIntervals.stream()
				.map(c -> c.getCost()).mapToDouble(Double::doubleValue)
				.max().getAsDouble();
			break;
		case AVERAGE:
			if (1 == subCostIntervals.stream()
				.map(c -> c.getCost()).mapToDouble(Double::doubleValue)
				.distinct().count()) {
				// avoid numerical issues in case of equal costs
				cost = subCostIntervals.stream().findFirst().get().getCost();
			} else {
				Duration duration = Duration.between(start, end);
				double intervalSeconds = duration.getSeconds()
						+ (duration.getNano() * 1E-9d);
				double subIntervalSeconds;
				for (CostInterval subCostInterval : subCostIntervals) {
					duration = Duration.between(subCostInterval.getLower(),
							subCostInterval.getUpper());
					subIntervalSeconds = duration.getSeconds()
							+ (duration.getNano() * 1E-9d);
					cost += subCostInterval.getCost()
							* subIntervalSeconds / intervalSeconds;
				}
			}
			break;
		}
		
		return cost;
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
		for (PlanningGrid grid : this.getAll()) {
			grid.time = time;
			grid.update();
		}
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
		for (PlanningGrid grid : this.getAll()) {
			grid.thresholdCost = thresholdCost;
			grid.updateVisibility();
		}
	}
	
	/**
	 * Updates this planning grid with all its children.
	 */
	public void refresh() {
		for (PlanningGrid grid : this.getAll()) {
			grid.update();
		}
	}
	
	/**
	 * Updates this planning grid for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @see DynamicEnvironment#refresh(Obstacle)
	 */
	@Override
	public void refresh(Obstacle obstacle) {
		if (this.obstacles.contains(obstacle)) {
			this.update();
			if (this.affectedChildren.containsKey(obstacle)) {
				for (PlanningGrid child : affectedChildren.get(obstacle)) {
					child.refresh(obstacle);
				}				
			}
		}
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
	 * Gets the obstacles within this planning grid.
	 * 
	 * @return the obstacles within this planning grid
	 *
	 * @see Environment#getObstacles()
	 */
	@Override
	public Set<Obstacle> getObstacles() {
		return Collections.unmodifiableSet(this.obstacles);
	}
	
	/**
	 * Embeds an obstacle into this planning grid.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see DynamicEnvironment#embed(Obstacle)
	 */
	@Override
	public boolean embed(Obstacle obstacle) {
		boolean embedded = false;
		
		if (this.hasGlobe()) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.addCostInterval(obstacle.getCostInterval());
				this.obstacles.add(obstacle);
				
				for (PlanningGrid child : this.getChildren()) {
					if (child.embed(obstacle)) {
						this.addAffectedChild(obstacle, child);
					}
				}
				
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
	 * @see DynamicEnvironment#unembed(Obstacle)
	 */
	@Override
	public boolean unembed(Obstacle obstacle) {
		boolean unembedded = false;
		
		if (this.isEmbedded(obstacle)) {
			this.removeCostInterval(obstacle.getCostInterval());
			this.obstacles.remove(obstacle);
			
			if (this.affectedChildren.containsKey(obstacle)) {
				for (PlanningGrid child : this.affectedChildren.get(obstacle)) {
					child.unembed(obstacle);
				}
				this.affectedChildren.remove(obstacle);
			}
			
			unembedded = true;
		}
		
		return unembedded;
	}
	
	/**
	 * Unembeds all obstacles from this planning grid.
	 * 
	 * @see DynamicEnvironment#unembedAll()
	 */
	@Override
	public void unembedAll() {
		Iterator<Obstacle> obstaclesIterator = this.obstacles.iterator();
		while (obstaclesIterator.hasNext()) {
			Obstacle obstacle = obstaclesIterator.next();
			this.removeCostInterval(obstacle.getCostInterval());
			obstaclesIterator.remove();
			
			if (this.affectedChildren.containsKey(obstacle)) {
				for (PlanningGrid child : this.affectedChildren.get(obstacle)) {
					child.unembed(obstacle);
				}
				this.affectedChildren.remove(obstacle);
			}
		}
	}
	
	/**
	 * Determines whether or not an obstacle is embedded in this planning grid.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this planning grid,
	 *         false otherwise
	 * 
	 * @see DynamicEnvironment#isEmbedded(Obstacle)
	 */
	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		return this.obstacles.contains(obstacle);
	}
	
	/**
	 * Adds an affected child to an obstacle embedding.
	 * 
	 * @param obstacle the obstacle of the embedding
	 * @param child the affected child
	 */
	private void addAffectedChild(Obstacle obstacle, PlanningGrid child) {
		if (this.affectedChildren.containsKey(obstacle)) {
			this.affectedChildren.get(obstacle).add(child);
		} else {
			ArrayList<PlanningGrid> children = new ArrayList<PlanningGrid>();
			children.add(child);
			this.affectedChildren.put(obstacle, children);
		}
	}
	
	/**
	 * Gets the children of this planning grid that are affected by an obstacle.
	 *  
	 * @param obstacle the obstacle
	 * @return the children of this planning grid that are affected by the
	 *         obstacle
	 * 
	 * @see DynamicHierarchicalEnvironment#getAffectedChildren(Obstacle)
	 */
	@Override
	public Set<? extends PlanningGrid> getAffectedChildren(Obstacle obstacle) {
		Set<PlanningGrid> affectedChildren = new HashSet<PlanningGrid>();
		
		if (this.hasGlobe()) {
			if ((null != obstacle) && this.hasChildren()) {
				for (PlanningGrid child : this.getChildren()) {
					if (child.intersects(obstacle.getExtent(this.getGlobe()))) {
						affectedChildren.add(child);
					}
				}
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return Collections.unmodifiableSet(affectedChildren);
	}
	
	/**
	 * Gets the waypoint positions of this planning grid that are affected by
	 * an obstacle.
	 * 
	 * @param obstacle the obstacle
	 * @return the waypoint positions of this planning grid that are affected
	 *         by the obstacle
	 * 
	 * @see DynamicEnvironment#getAffectedWaypointPositions(Obstacle)
	 */
	@Override
	public Set<Position> getAffectedWaypointPositions(Obstacle obstacle) {
		Set<Position> affectedWaypointPositions = new HashSet<Position>();
		
		if (null != obstacle) {
			if (this.hasChildren()) {
				for (PlanningGrid affectedChild : this.getAffectedChildren(obstacle)) {
					if (affectedChild.hasChildren()) {
						affectedWaypointPositions.addAll(affectedChild.getAffectedWaypointPositions(obstacle));
					} else {
						affectedWaypointPositions.addAll(Arrays.asList(affectedChild.getCornerPositions()));
					}
				}
			} else {
				if (this.intersects(obstacle.getExtent(this.getGlobe()))) {
					affectedWaypointPositions.addAll(Arrays.asList(this.getCornerPositions()));
				}
			}
		}
		
		return Collections.unmodifiableSet(affectedWaypointPositions);
	}
	
	/**
	 * Gets the waypoint positions of this planning grid that are affected by
	 * obstacles.
	 * 
	 * @param obstacles the obstacles
	 * @return the waypoint positions of this planning grid that are affected
	 *         by the obstacles
	 * 
	 * @see DynamicEnvironment#getAffectedWaypointPositions(Set)
	 */
	@Override
	public Set<Position> getAffectedWaypointPositions(Set<Obstacle> obstacles) {
		Set<Position> affectedWaypointPositions = new HashSet<Position>();
		
		if (null != obstacles) {
			for (Obstacle obstacle : obstacles) {
				affectedWaypointPositions.addAll(this.getAffectedWaypointPositions(obstacle));
			}
		}
		
		return Collections.unmodifiableSet(affectedWaypointPositions);
	}
	
	/**
	 * Adds the specified number of children on each axis to this planning grid
	 * and propagates existing obstacle embeddings.
	 * 
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 * 
	 * @see CubicGrid#addChildren(int, int, int)
	 */
	@Override
	public void addChildren(int rCells, int sCells, int tCells) {
		super.addChildren(rCells, sCells, tCells);
		
		for (PlanningGrid child : this.getChildren()) {
			// initialize children
			child.setGlobe(this.globe);
			child.setTime(this.time);
			child.setThreshold(this.thresholdCost);
			child.update();
			
			// propagate obstacle embeddings
			for (Obstacle obstacle : this.obstacles) {
				if (obstacle instanceof ObstacleCylinder) {
					if (child.embed((ObstacleCylinder) obstacle)) {
						this.addAffectedChild(obstacle, child);
					}
				}
				// TODO: implement propagations for other extents
			}
		}
	}

	/**
	 * Removes all children from this planning grid.
	 * 
	 * @see CubicGrid#removeChildren()
	 */
	@Override
	public void removeChildren() {
		super.removeChildren();
		// remove affected children of all obstacle embeddings
		this.affectedChildren.clear();
	}
	
	/**
	 * Gets the children of this planning grid.
	 * 
	 * @return the children of this planning grid
	 * 
	 * @see HierarchicalEnvironment#getChildren()
	 * @see CubicGrid#getChildren()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> getChildren() {
		return (Set<PlanningGrid>) super.getChildren();
	}
	
	/**
	 * Gets a particular child of this planning grid if present.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * 
	 * @return the particular child of this planning grid if present,
	 *         null otherwise
	 * 
	 * @see CubicGrid#getChild(int, int, int)
	 */
	@Override
	public PlanningGrid getChild(int r, int s, int t) {
		return (PlanningGrid) super.getChild(r, s, t);
	}
	
	/**
	 * Gets the parent of this planning grid if present.
	 * 
	 * @return the parent of this planning grid if present,
	 *         null otherwise
	 * 
	 * @see HierarchicalEnvironment#getParent()
	 * @see CubicGrid#getParent()
	 */
	@Override
	public PlanningGrid getParent() {
		return (PlanningGrid) super.getParent();
	}
	
	/**
	 * Gets all planning grids associated with this planning grid.
	 * 
	 * @return all planning grids associated with this planning grid
	 * 
	 * @see HierarchicalEnvironment#getAll()
	 * @see CubicGrid#getAll()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> getAll() {
		return (Set<PlanningGrid>) super.getAll();
	}
	
	/**
	 * Determines whether or not this planning grid is refined, that is,
	 * has children.
	 * 
	 * @return true if this planning grid is refined, false otherwise
	 * 
	 * @see MultiResolutionEnvironment#isRefined()
	 */
	@Override
	public boolean isRefined() {
		return this.hasChildren();
	}
	
	/**
	 * Gets the resolution of this planning grid.
	 * 
	 * @return the resolution of this planning grid
	 * 
	 * @see MultiResolutionEnvironment#getResolution()
	 */
	@Override
	public double getResolution() {
		double resolution = 1d;
		
		if (this.hasChildren()) {
			resolution = Math.cbrt(this.getChildren().size());
		}
		
		return resolution;
	}
	
	/**
	 * Sets the resolution of this planning grid.
	 * 
	 * @param resolution the resolution to be set
	 * 
	 * @throws IllegalArgumentException if the resolution is not positive
	 * 
	 * @see MultiResolutionEnvironment#setResolution(double)
	 */
	@Override
	public void setResolution(double resolution) {
		if (1d > resolution) {
			throw new IllegalArgumentException("invalid resolution");
		} else {
			this.removeChildren();
			if (1d < resolution) {
				this.addChildren((int) Math.ceil(resolution));
				for (PlanningGrid grid : this.getChildren()) {
					for (StructuralChangeListener listener : this.listeners) {
						grid.addStructuralChangeListener(listener);
					}
				}
			}
			this.notifyStructuralChangeListeners();
		}
	}
	
	/**
	 * Refines this planning grid by a refinement factor, that is, adds
	 * children to this planning grid.
	 * 
	 * @param factor the refinement factor
	 * 
	 * @throws IllegalArgumentException if the refinement factor is less than 2
	 * 
	 * @see MultiResolutionEnvironment#refine(int)
	 */
	@Override
	public void refine(int factor) {
		if (2 > factor) {
			throw new IllegalArgumentException("invalid refinement factor");
		} else {
			int resolution = (int) this.getResolution() * factor;
			this.setResolution(resolution);
		}
	}
	
	/**
	 * Coarsens this planning grid by a coarsening factor, that is, removes
	 * children from this planning grid.
	 *
	 * @param factor the coarsening factor
	 * 
	 * @throws IllegalArgumentException if the coarsening factor is less than 2
	 * 
	 * @see MultiResolutionEnvironment#coarsen
	 */
	@Override
	public void coarsen(int factor) {
		if (2 > factor) {
			throw new IllegalArgumentException("invalid coarsening factor");
		} else if (this.hasParent()) {
			int resolution = (int) this.getResolution() / factor;
			this.setResolution(resolution);
		}
	}
	
	/**
	 * Looks up the planning grid cells (maximum eight) containing a specified
	 * vector in world model coordinates considering numerical inaccuracies.
	 * 
	 * @param modelVector the vector in world model coordinates
	 * 
	 * @return the planning grid cells containing the specified vector
	 * 
	 * @see CubicGrid#lookupCells(Vec4)
	 */
	@Override
	@SuppressWarnings("unchecked")
	public Set<? extends PlanningGrid> lookupCells(Vec4 modelVector) {
		return (Set<PlanningGrid>) super.lookupCells(modelVector);
	}
	
	/**
	 * Looks up the planning grid cells (maximum eight) containing a specified
	 * position in globe coordinates considering numerical inaccuracies.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the planning grid cells containing the specified position
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see CubicGrid#lookupCells(Vec4)
	 */
	@SuppressWarnings("unchecked")
	public Set<? extends PlanningGrid> lookupCells(Position position) {
		Set<PlanningGrid> cells = null;
		
		if (this.hasGlobe()) {
			cells = (Set<PlanningGrid>) super.lookupCells(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return cells;
	}
	
	/**
	 * Finds all cells of this planning grid that satisfy a specified predicate.
	 * A full recursive search is performed considering only non-parent cells.
	 * 
	 * @param predicate the predicate
	 * 
	 * @return the cells of this planning grid that satisfy a predicate
	 * 
	 * @see CubicGrid#findCells(Predicate)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> findCells(Predicate<RegularGrid> predicate) {
		return (Set<PlanningGrid>) super.findCells(predicate);
	}
	
	/**
	 * Finds all cells of this planning grid that satisfy a specified predicate
	 * taking a specified hierarchical depth into account. A zero depth does
	 * not consider any children. A negative depth performs a full recursive
	 * search and considers non-parent cells only.
	 * 
	 * @param predicate the predicate
	 * @param depth the hierarchical depth
	 * 
	 * @return the cells of this planning grid that satisfy a predicate taking
	 *         the hierarchical depth into account
	 * 
	 * @see CubicGrid#findCells(Predicate, int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> findCells(Predicate<RegularGrid> predicate, int depth) {
		return (Set<PlanningGrid>) super.findCells(predicate, depth);
	}
	
	/**
	 * Gets the intersection positions of a straight leg with the cells of this
	 * planning grid. A full recursive search is performed considering only
	 * non-parent cells.
	 * 
	 * @param origin the origin of the leg in globe coordinates
	 * @param destination the destination of the leg in globe coordinates
	 * 
	 * @return the intersection positions of the straight leg with the cells of
	 *         this planning grid
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see CubicGrid#getIntersectionVectors(Vec4, Vec4)
	 */
	public Iterable<? extends Position> getIntersectedPositions(Position origin, Position destination) {
		if (this.hasGlobe()) {
			return
				super.getIntersectionVectors(
					this.globe.computePointFromPosition(origin),
					this.globe.computePointFromPosition(destination))
				.stream()
				.map(this.globe::computePositionFromPoint)
				.collect(Collectors.toCollection(LinkedHashSet<Position>::new));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Determines whether or not a position is a waypoint position in this
	 * planning grid.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is a waypoint position in this planning
	 *         grid, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Environment#isWaypointPosition(Position)
	 * @see RegularGrid#isWaypointVector(Vec4)
	 */
	@Override
	public boolean isWaypointPosition(Position position) {
		if (this.hasGlobe()) {
			return super.isWaypointVector(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Gets the adjacent waypoint positions of a position in this planning
	 * grid.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the adjacent waypoint positions of the position in this planning
	 *         grid, or the waypoint position itself
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Environment#getAdjacentWaypointPositions(Position)
	 * @see RegularGrid#getAdjacentWaypointVectors(Vec4)
	 */
	@Override
	public Set<Position> getAdjacentWaypointPositions(Position position) {
		if (this.hasGlobe()) {
			Set<Vec4> waypointVectors = super.getAdjacentWaypointVectors(this.globe.computePointFromPosition(position));
			return waypointVectors
					.stream()
					.map(this.globe::computePositionFromPoint)
					.collect(Collectors.toSet());
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Determines whether or not a position is adjacent to a waypoint position
	 * in this planning grid.
	 * 
	 * @param position the position in globe coordinates
	 * @param waypointPosition the waypoint position in globe coordinates
	 * 
	 * @return true if the position is adjacent to the waypoint position in
	 *         this planning grid, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Environment#isAdjacentWaypointPosition(Position, Position)
	 * @see RegularGrid#isAdjacentWaypointVector(Vec4, Vec4)
	 */
	@Override
	public boolean isAdjacentWaypointPosition(Position position, Position waypointPosition) {
		if (this.hasGlobe()) {
			return super.isAdjacentWaypointVector(
					this.globe.computePointFromPosition(position),
					this.globe.computePointFromPosition(waypointPosition));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Gets the neighbors of this planning grid. A full recursive search is
	 * performed considering only non-parent neighbors.
	 * 
	 * @return the non-parent neighbors of this planning grid
	 * 
	 * @see CubicGrid#getNeighbors()
	 * @see HierarchicalEnvironment#getNeighbors()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> getNeighbors() {
		return (Set<PlanningGrid>) super.getNeighbors();
	}
	
	/**
	 * Gets the neighbors of this planning grid taking a specified hierarchical
	 * depth into account. A zero depth does not consider any neighboring
	 * children. A negative depth performs a full recursive search and
	 * considers non-parent neighbors only.
	 * 
	 * @param depth the hierarchical depth for finding neighbors
	 * 
	 * @return the neighbors of this planning grid
	 * 
	 * @see HierarchicalEnvironment#getNeighbors()
	 * @see CubicGrid#getNeighbors(int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> getNeighbors(int depth) {
		return (Set<PlanningGrid>) super.getNeighbors(depth);
	}
	
	/**
	 * Determines whether or not this planning grid is a neighbor of another
	 * hierarchical environment.
	 * 
	 * @param neighbor the potential neighbor of this planning grid
	 * 
	 * @return true if this planning grid is a neighbor of the other
	 *         hierarchical environment, false otherwise
	 * 
	 * @see HierarchicalEnvironment#areNeighbors(HierarchicalEnvironment)
	 * @see RegularGrid#areNeighbors(RegularGrid)
	 */
	@Override
	public boolean areNeighbors(HierarchicalEnvironment neighbor) {
		return this.getNeighbors().contains(neighbor);
	}
	
	/**
	 * Gets the neighbors of a position in this planning grid. A full recursive
	 * search is performed considering non-parent cells only.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the neighbors of the position in this planning grid
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see CubicGrid#getNeighbors(Vec4)
	 * @see Environment#getNeighbors(Position)
	 */
	@Override
	public Set<Position> getNeighbors(Position position) {
		Set<Position> neighbors = new HashSet<Position>();
		
		// TODO: coordinate transformations might be too expensive for planning
		// TODO: planning could be based on Vec4 with a final transformation of the route
		
		if (this.hasGlobe()) {
			Set<Vec4> neighborVectors = super.getNeighbors(this.globe.computePointFromPosition(position));
			for (Vec4 neighbor : neighborVectors) {
				neighbors.add(this.globe.computePositionFromPoint(neighbor));
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}
		
		return neighbors;
	}
	
	/**
	 * Determines whether or not two positions are neighbors in this planning
	 * grid.
	 * 
	 * @param position the position
	 * @param neighbor the potential neighbor of the position
	 * 
	 * @return true if the two positions are neighbors, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Environment#areNeighbors(Position, Position)
	 * @see RegularGrid#areNeighbors(Vec4, Vec4)
	 */
	@Override
	public boolean areNeighbors(Position position, Position neighbor) {
		if (this.hasGlobe()) {
			return super.areNeighbors(
					this.globe.computePointFromPosition(position),
					this.globe.computePointFromPosition(neighbor));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Gets the distance between two positions in this planning grid.
	 * 
	 * @param position1 the first position in globe coordinates
	 * @param position2 the second position in globe coordinates
	 * 
	 * @return the distance between the two positions in this planning grid
	 * 
	 * @throws IllegalStateException if this planning grid has no globe
	 * 
	 * @see Environment#getDistance(Position, Position)
	 */
	@Override
	public double getDistance(Position position1, Position position2) {
		if (this.hasGlobe()) {
			ArrayList<Position> positions = new ArrayList<Position>();
			positions.add(position1);
			positions.add(position2);
			LengthMeasurer measurer = new LengthMeasurer(positions);
			measurer.setPathType(AVKey.LINEAR);
			measurer.setFollowTerrain(false);
			return measurer.getLength(this.getGlobe());
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Gets the great circle distance between two positions in this planning
	 * grid.
	 * 
	 * @param position1 the first position in globe coordinates
	 * @param position2 the second position in globe coordinates
	 * 
	 * @return the great circle distance between the two positions in this
	 *         planning grid
	 * 
	 * @throws IllegalStateException if this planning grid has no globe
	 * 
	 * @see Environment#getGreatCircleDistance(Position, Position)
	 */
	@Override
	public double getGreatCircleDistance(Position position1, Position position2) {
		if (this.hasGlobe()) {
			ArrayList<Position> positions = new ArrayList<Position>();
			positions.add(position1);
			positions.add(position2);
			LengthMeasurer measurer = new LengthMeasurer(positions);
			measurer.setPathType(AVKey.GREAT_CIRCLE);
			measurer.setFollowTerrain(false);
			return measurer.getLength(this.getGlobe());
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Gets the normalized distance between two positions in this planning grid.
	 * 
	 * @param position1 the first position in globe coordinates
	 * @param position2 the second position in globe coordintates
	 * 
	 * @return the normalized distance between the two positions in this
	 *         planning grid
	 * 
	 * @see Environment#getNormalizedDistance(Position, Position)
	 */
	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		return this.getDistance(position1, position2) / this.getNormalizer();
	}
	
	/**
	 * Converts a normalized distance to a distance within this planning grid.
	 * 
	 * @param normalizedDistance the normalized distance
	 * 
	 * @return the distance
	 * 
	 * @see Environment#toDistance(double)
	 */
	@Override
	public double toDistance(double normalizedDistance) {
		return normalizedDistance * this.getNormalizer();
	}
	
	/**
	 * Converts a distance to a normalized distance within this planning grid.
	 * 
	 * @param distance the distance
	 * 
	 * @return the normalized distance
	 * 
	 * @see Environment#toNormalizedDistance(double)
	 */
	@Override
	public double toNormalizedDistance(double distance) {
		return distance / this.getNormalizer();
	}
	
	/**
	 * Gets the volume of this planning grid.
	 * 
	 * @return the volume of this planning grid
	 * 
	 * @see Environment#getVolume()
	 */
	@Override
	public double getVolume() {
		// TODO: use globe for conversion?
		return this.getRLength() * this.getSLength() * this.getTLength();
	}
	
	/**
	 * Gets the step cost from an origin to a destination position within this
	 * planning grid between a start and an end time given a cost policy and
	 * risk policy.
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
	@Override
	public double getStepCost(
			Position origin, Position destination,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		
		double stepCost = 0d;
		
		// compute participating cells
		Set<? extends PlanningGrid> segmentCells = this.lookupCells(origin);
		segmentCells.retainAll(this.lookupCells(destination));
		
		// an invalid step results in infinite costs
		if (segmentCells.isEmpty()) {
			return Double.POSITIVE_INFINITY;
		}
		
		List<Double> costs = new ArrayList<Double>();
		
		// compute initial distance cost
		// explicit distance cost computation is required if neighboring
		// cells are of different size (different level in the hierarchy)
		double distance = this.getNormalizedDistance(origin, destination);
		
		// TODO: consider environment air-data intervals
		// compute cost of each adjacent cell
		for (PlanningGrid segmentCell : segmentCells) {
			// obtain (weighted) cost of the cell
			double cellCost = segmentCell.getCost(start, end, costPolicy, riskPolicy);
			// boost cell cost if local risk is not acceptable
			if (riskPolicy.satisfies(cellCost - this.getBaseCost())) {
				costs.add(distance * cellCost);
			} else {
				costs.add(Double.POSITIVE_INFINITY);
			}
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
	
	/**
	 * Gets the leg cost from an origin to a destination position within this
	 * planning grid between a start and an end time given a cost policy and
	 * risk policy.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the leg cost from the origin to the destination position
	 * 
	 * @see Environment#getLegCost(Position, Position, ZonedDateTime, ZonedDateTime, CostPolicy, RiskPolicy)
	 */
	@Override
	public double getLegCost(
			Position origin, Position destination,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		
		double legCost = Double.POSITIVE_INFINITY;
		
		// compute leg performance
		Duration legDuration = Duration.between(start, end);
		double legDistance = this.getDistance(origin, destination);
		double legSpeed = legDistance / legDuration.getSeconds();
		
		// compute all intersection positions on this straight leg
		Iterator<? extends Position> positionIterator =
				this.getIntersectedPositions(origin, destination).iterator();
		
		// compute the cost of each leg segment
		if (positionIterator.hasNext()) {
			legCost = 0d;
			ZonedDateTime currentEto = start;
			Position current = positionIterator.next();
			
			while (positionIterator.hasNext()) {
				Position next = positionIterator.next();
				
				// compute step performance
				double stepDistance = this.getDistance(current, next);
				Duration stepDuration =  Duration.ofSeconds(Math.round((stepDistance / legSpeed)));
				ZonedDateTime nextEto = currentEto.plus(stepDuration);
				
				legCost += this.getStepCost(
						current, next, currentEto, nextEto, costPolicy, riskPolicy);
				
				currentEto = nextEto;
				current = next;
			}
		}
		
		return legCost;
	}
	
	/**
	 * Determines whether or not a straight leg of two positions collides with
	 * terrain of the globe of this planning grid.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * 
	 * @return true if the straight leg collides with terrain, false otherwise
	 * 
	 * @see Environment#collidesTerrain(Position, Position)
	 */
	@Override
	public boolean collidesTerrain(Position origin, Position destination) {
		boolean collidesTerrain = true;
		
		if (!this.isInsideGlobe(origin) && !this.isInsideGlobe(destination)) {
			HighResolutionTerrain terrain = new HighResolutionTerrain(
					this.getGlobe(), this.getResolution());
			// TODO: check position altitudes (ASL versus AGL)
			// TODO: include safe height and distance
			collidesTerrain = (null != terrain.intersect(origin, destination));
		}
		
		return collidesTerrain;
	}
	
	/**
	 * Gets the leg cost from the center of this planning grid to the center
	 * of another environment between a start and an end time given a
	 * cost policy and risk policy. 
	 * 
	 * @param destination the destination environment
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the leg cost from the center of this environment to the center
	 *         of the destination environment
	 * 
	 * @see HierarchicalEnvironment#getLegCost(HierarchicalEnvironment, ZonedDateTime, ZonedDateTime, CostPolicy, RiskPolicy)
	 */
	@Override
	public double getLegCost(
			HierarchicalEnvironment destination,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		
		double legCost = Double.POSITIVE_INFINITY;
		
		// compute leg performance
		Duration legDuration = Duration.between(start, end);
		double legDistance = this.getDistance(
				this.getCenterPosition(), destination.getCenterPosition());
		double legSpeed = legDistance / legDuration.getSeconds();
		
		// compute all intersection positions on this straight leg
		Iterator<? extends Position> positionIterator =
				this.getIntersectedPositions(
						this.getCenterPosition(),
						destination.getCenterPosition()).iterator();
		
		// compute the cost of each leg segment
		if (positionIterator.hasNext()) {
			legCost = 0d;
			ZonedDateTime currentEto = start;
			Position current = positionIterator.next();
			
			while (positionIterator.hasNext()) {
				Position next = positionIterator.next();
				
				// compute step performance
				double stepDistance = this.getDistance(current, next);
				Duration stepDuration =  Duration.ofSeconds(Math.round((stepDistance / legSpeed)));
				ZonedDateTime nextEto = currentEto.plus(stepDuration);
				
				legCost += this.getStepCost(
						current, next, currentEto, nextEto, costPolicy, riskPolicy);
				
				currentEto = nextEto;
				current = next;
			}
		}
		
		return legCost;
	}
	
	/**
	 * Adds a structural change listener to this planning grid.
	 * 
	 * @param listener the structural change listener to be added
	 * 
	 * @see StructuredEnvironment#addStructuralChangeListener(StructuralChangeListener)
	 */
	@Override
	public void addStructuralChangeListener(StructuralChangeListener listener) {
		this.listeners.add(listener);
		for (PlanningGrid grid : this.getChildren()) {
			grid.addStructuralChangeListener(listener);
		}
	}
	
	/**
	 * Removes a structural change listener from this planning grid.
	 * 
	 * @param listener the structural change listener to be removed
	 * 
	 * @see StructuredEnvironment#removeStructuralChangeListener(StructuralChangeListener)
	 */
	@Override
	public void removeStructuralChangeListener(StructuralChangeListener listener) {
		this.listeners.remove(listener);
		for (PlanningGrid grid : this.getChildren()) {
			grid.removeStructuralChangeListener(listener);
		}
	}
	
	/**
	 * Notifies the structural change listeners of this planning grid.
	 * 
	 * @see StructuredEnvironment#notifyStructuralChangeListeners()
	 */
	@Override
	public void notifyStructuralChangeListeners() {
		for (StructuralChangeListener listener : this.listeners) {
			listener.notifyStructuralChange();
		}
	}
	
}
