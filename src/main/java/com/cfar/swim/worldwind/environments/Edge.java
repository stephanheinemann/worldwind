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
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.geom.LineSegment;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.WeightedCostInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an edge of a roadmap or tree within a continuum based on two
 * sampled positions featuring temporal costs.
 * 
 * @author Henrique Ferreira
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class Edge extends LineSegment implements TimedRenderable, ThresholdRenderable {
	
	// TODO: Consider edges to be dynamic, multi-resolution environments itself
	// bounded by the sphere with its center at the center of the line segment
	// and its radius half the segment length. Each Edge (Sphere) has a parent
	// which is a planning continuum with either a Sphere (intermediate) or Box
	// (top level) shape. Alternatively all continuums could be ellipsoids with
	// the line segment end points at their foci.
	
	/** the continuum of this edge */
	private PlanningContinuum continuum = null;
	
	/** the cost interval tree encoding temporal costs of this edge */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals =
			new IntervalTree<ChronoZonedDateTime<?>>(CostInterval.comparator);
	
	/** the current accumulated active cost of this edge */
	private double activeCost;
	
	/**
	 * Constructs a new edge within a planning continuum based on two end
	 * positions.
	 * 
	 * @param continuum the planning continuum containing this edge
	 * @param first the first end position of this edge
	 * @param second the second end position of this edge
	 */
	public Edge(PlanningContinuum continuum, Position first, Position second) {
		super(continuum.getGlobe().computePointFromPosition(first),
				continuum.getGlobe().computePointFromPosition(second));
		this.continuum = continuum;
		this.initCostIntervals();
	}

	/**
	 * Gets the first end position of this edge.
	 * 
	 * @return the first end position of this edge
	 */
	public Position getFirstPosition() {
		return this.continuum.getGlobe().computePositionFromPoint(this.getFirst());
	}

	/**
	 * Sets the first end position of this edge.
	 * 
	 * @param first the first end position of this edge
	 */
	public void setFirstPosition(Position first) {
		this.setFirst(this.continuum.getGlobe().computePointFromPosition(first));
		this.initCostIntervals();
	}

	/**
	 * Gets the second end position of this edge.
	 * 
	 * @return the second end position of this edge
	 */
	public Position getSecondPosition() {
		return this.continuum.getGlobe().computePositionFromPoint(this.getSecond());
	}

	/**
	 * Sets the second end position of this edge.
	 * 
	 * @param second the second end position of this edge
	 */
	public void setSecondPosition(Position second) {
		this.setSecond(this.continuum.getGlobe().computePointFromPosition(second));
		this.initCostIntervals();
	}
	
	/**
	 * Gets the other end position of this edge.
	 * 
	 * @param position one end position of this edge
	 * 
	 * @return the other end position of this edge, null otherwise
	 */
	public Position getOtherPosition(Position position) {
		return this.continuum.getGlobe().computePositionFromPoint(
				this.getOther(this.continuum.getGlobe().computePointFromPosition(position)));
	}
	
	/**
	 * Determines whether or not a position is an end position of this edge.
	 * 
	 * @param position the position to be checked
	 * 
	 * @return true if the position is an end position of this edge,
	 *         false otherwise
	 */
	public boolean isEndPosition(Position position) {
		return this.isEndpoint(this.continuum.getGlobe().computePointFromPosition(position));
	}
	
	/**
	 * Initializes the cost interval tree of this edge.
	 */
	public void initCostIntervals() {
		this.clearCostIntervals();
		for (Obstacle obstacle : this.continuum.getObstacles()) {
			if (this.intersects(obstacle.getExtent(this.continuum.getGlobe()))) {
				this.addCostInterval(obstacle.getCostInterval());
			}
		}
		this.update();
	}
	
	/**
	 * Clears the cost interval tree of this edge.
	 */
	public void clearCostIntervals() {
		this.costIntervals.clear();
		this.update();
	}
	
	/**
	 * Adds a cost interval to the cost interval tree of this edge.
	 * 
	 * @param costInterval the cost interval to be added
	 */
	public void addCostInterval(CostInterval costInterval) {
		this.costIntervals.add(costInterval);
		this.update();
	}

	/**
	 * Removes a cost interval from the cost interval tree of this edge.
	 * 
	 * @param costInterval the cost interval to be removed
	 */
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
	 */
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		return this.costIntervals.searchInterval(new CostInterval(null, time));
	}

	/**
	 * Gets all cost intervals that are active during a specified time interval.
	 * 
	 * @param start the start time of the time interval
	 * @param end the end time of the time interval
	 * 
	 * @return all cost intervals that are active during the specified time interval
	 */
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		return this.costIntervals.searchInterval(new CostInterval(null, start, end));
	}
	
	/**
	 * Gets the accumulated cost of this edge at a specified time instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return the accumulated cost of this edge at the specified time instant
	 */
	public double getCost(ZonedDateTime time) {
		return this.getCost(time, time);
	}
	
	/**
	 * Gets the accumulated cost of this edge within a specified time span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the accumulated cost of this edge within the specified time span
	 *
	 */
	public double getCost(ZonedDateTime start, ZonedDateTime end) {
		double cost = this.continuum.getBaseCost(); // simple cost of normalized distance
		
		Set<String> costIntervalIds = new HashSet<String>();
		// add all (weighted) cost of the cell
		List<Interval<ChronoZonedDateTime<?>>> intervals = this.getCostIntervals(start, end);
		for (Interval<ChronoZonedDateTime<?>> interval : intervals) {
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;
				
				// only add costs of different overlapping cost intervals
				if (!costIntervalIds.contains(costInterval.getId())) {
					costIntervalIds.add(costInterval.getId());
					
					// TODO: implement a proper weighted cost calculation normalized from 0 to 100
					// TODO: the weight is affected by severity (reporting method) and currency (reporting time)
					
					if ((interval instanceof WeightedCostInterval)) {
						cost += ((WeightedCostInterval) interval).getWeightedCost();
					} else {
						cost += costInterval.getCost();
					}
				}	
			}
		}
		
		return cost;
	}
	
	// TODO: review calculate cost methods
	
	/**
	 * Calculates the cost of this edge within a specified time span by searching
	 * the cost interval tree.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * @param costPolicy the cost policy of the planner
	 * @param riskPolicy the risk policy of the planner
	 * 
	 * @return the accumulated cost of this edge within the specified time span
	 */
	public double calculateCost(ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy, RiskPolicy riskPolicy) {
		double cost = 1d; // simple cost of normalized distance
		List<Double> costList = new ArrayList<Double>();

		Set<String> costIntervalIds = new HashSet<String>();
		// add all (weighted) cost of the cell
		List<Interval<ChronoZonedDateTime<?>>> intervals = this.getCostIntervals(start, end);
		for (Interval<ChronoZonedDateTime<?>> interval : intervals) {
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;

				// only add costs of different overlapping cost intervals
				if (!costIntervalIds.contains(costInterval.getId())) {
					costIntervalIds.add(costInterval.getId());

					if ((interval instanceof WeightedCostInterval)) {
						costList.add(((WeightedCostInterval) interval).getWeightedCost());
					} else {
						costList.add(costInterval.getCost());
					}
				}
			}
		}
		// TODO: Review cost policy implementation
		if (!costList.isEmpty()) {
			// cost is computed based on the minimum/average/maximum cost of all obstacles
			// times the number of obstacles that affect this edge
			/*
			 * switch (costPolicy) { case MINIMUM: cost =
			 * costList.stream().mapToDouble(Double::doubleValue).min().getAsDouble() *
			 * costList.size(); break; case MAXIMUM: cost =
			 * costList.stream().mapToDouble(Double::doubleValue).max().getAsDouble() *
			 * costList.size(); break; case AVERAGE: cost =
			 * costList.stream().mapToDouble(Double::doubleValue).average().getAsDouble() *
			 * costList.size(); break; }
			 */
			cost = costList.stream().mapToDouble(f -> f.doubleValue()).sum();
		}

		// Risk Policy implementation
		if (!riskPolicy.satisfies(cost))
			cost = Double.POSITIVE_INFINITY;

		return cost;
	}

	/**
	 * Calculates the cost of this edge within a specified time span by searching
	 * the cost interval tree.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the accumulated cost of this edge within the specified time span
	 */
	public double calculateCostNew(ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {

		double cost = 1d; // simple cost of normalized distance

		Set<ZonedDateTime> times = new HashSet<ZonedDateTime>();
		times.add(start);
		times.add(end);

		// add all (weighted) cost of the cell
		List<Interval<ChronoZonedDateTime<?>>> intervals = this.getCostIntervals(start, end);

		// For every interval get sub-Interval times
		ZonedDateTime lower, upper;
		for (Interval<ChronoZonedDateTime<?>> interval : intervals) {
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;
				lower = costInterval.getLower();
				upper = costInterval.getUpper();

				// Add lower limit to times if it is higher than start
				if (lower.isAfter(start))
					times.add(costInterval.getLower());
				// Add upper limit to times if it is lower than end
				if (upper.isBefore(end))
					times.add(costInterval.getUpper());

			}
		}

		List<ZonedDateTime> sortedTimes = times.stream().sorted().collect(Collectors.toList());
		Iterator<ZonedDateTime> iterator = sortedTimes.iterator();
		List<Double> costListTemporal = new ArrayList<Double>();
		ZonedDateTime startPart = iterator.next(), endPart;
		double costPart = 1d, costAux = 0d;

		// For every sub-Interval calculate cost
		while (iterator.hasNext()) {
			endPart = iterator.next();
			costPart = 1d;
			costAux = 0d;
			for (Interval<ChronoZonedDateTime<?>> interval : intervals) {
				if (interval instanceof CostInterval) {
					CostInterval costInterval = (CostInterval) interval;
					lower = costInterval.getLower();
					upper = costInterval.getUpper();

					// Only consider intervals that fully contain this sub-interval
					if (!lower.isAfter(startPart) && !upper.isBefore(endPart)) {
						if ((interval instanceof WeightedCostInterval)) {
							costAux += ((WeightedCostInterval) interval).getWeightedCost();
						} else {
							costAux += costInterval.getCost();
						}
					}
				}
			}
			costPart = costAux == 0d ? costPart : costAux;

			// Risk Policy implementation
			if (!riskPolicy.satisfies(costPart))
				costPart = Double.POSITIVE_INFINITY;

			costListTemporal.add(costPart);
			startPart = endPart;
		}

		// Cost Policy implementation to chose time interval of interest
		switch (costPolicy) {
		case MINIMUM:
			cost = costListTemporal.stream().mapToDouble(Double::doubleValue).min().getAsDouble();
			break;
		case MAXIMUM:
			cost = costListTemporal.stream().mapToDouble(Double::doubleValue).max().getAsDouble();
			break;
		case AVERAGE:
			// if the cost is the same during all sub intervals, avoids numerical errors
			if (costListTemporal.stream().mapToDouble(Double::doubleValue).distinct().limit(2).count() <= 1) {
				cost = costListTemporal.stream().mapToDouble(Double::doubleValue).average().getAsDouble();
				break;
			}
			ZonedDateTime startAux = sortedTimes.get(0), endAux;
			double duration;
			for (int i = 0; i < costListTemporal.size(); i++) {
				endAux = sortedTimes.get(i + 1);
				duration = Duration.between(startAux, endAux).getSeconds();
				cost += costListTemporal.get(i) * duration;
				startAux = endAux;
			}
			duration = Duration.between(start, end).getSeconds();
			cost = cost / duration;

			break;
		}

		return cost;
	}
	
	/**
	 * Sets the threshold of this threshold renderable edge.
	 * 
	 * @param threshold the threshold of this threshold renderable edge
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double threshold) {
		this.continuum.setThreshold(threshold);
		this.update();
	}
	
	/**
	 * Gets the threshold of this threshold renderable edge.
	 * 
	 * @return the threshold of this threshold renderable edge
	 * 
	 * @see ThresholdRenderable#getThreshold()
	 */
	@Override
	public double getThreshold() {
		return this.continuum.getThreshold();
	}
	
	/**
	 * Gets the time of this timed renderable edge.
	 * 
	 * @return the time of this timed renderable edge
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.continuum.getTime();
	}
	
	/**
	 * Sets the time of this timed renderable edge.
	 * 
	 * @param time the time of this timed renderable edge
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		this.continuum.setTime(time);
		this.update();
	}
	
	/**
	 * Updates this edge.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
	}
	
	/**
	 * Updates the accumulated active cost of this edge.
	 */
	protected void updateActiveCost() {
		this.activeCost = this.getCost(this.getTime());
	}
	
	/**
	 * Updates the appearance of this edge.
	 */
	protected void updateAppearance() {
		Color activeColor = ObstacleColor.getColor(this.activeCost);
		float red = activeColor.getRed() / 255.0f;
		float green = activeColor.getGreen() / 255.0f;
		float blue = activeColor.getBlue() / 255.0f;
		float alpha = activeColor.getAlpha() / 255.0f;
		this.setColor(red, green, blue, alpha);
	}
	
	/**
	 * Updates the visibility of this edge.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.getThreshold());
	}
	
}