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

import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.planning.WeightedCostInterval;

import gov.nasa.worldwind.geom.Position;

/**
 * Extends Waypoint to add a costIntervalTree
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public class SampledWaypoint extends Waypoint {

	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(
			CostInterval.comparator);

	/**
	 * Creates a new sampled waypoint based on a given position
	 * 
	 * @param position the position in global coordinates
	 */
	public SampledWaypoint(Position position) {
		super(position);
	}

	/**
	 * Creates a new sampled waypoint based on a given position and the associated
	 * costIntervals
	 * 
	 * @param position the position in global coordinates
	 * @param costIntervals the interval tree with cost intervals for this position
	 */
	public SampledWaypoint(Position position, IntervalTree<ChronoZonedDateTime<?>> costIntervals) {
		super(position);
		this.costIntervals = costIntervals;
	}

	/**
	 * Gets the interval tree with cost intervals.
	 * 
	 * @return the costIntervals
	 */
	public IntervalTree<ChronoZonedDateTime<?>> getCostIntervals() {
		return costIntervals;
	}

	/**
	 * Sets the interval tree with cost intervals.
	 * 
	 * @param costIntervals the costIntervals to set
	 */
	public void setCostIntervals(IntervalTree<ChronoZonedDateTime<?>> costIntervals) {
		this.costIntervals = costIntervals;
	}

	/**
	 * Adds a cost interval to this sampled waypoint.
	 * 
	 * @param costInterval the cost interval to be added
	 */
	public void addCostInterval(CostInterval costInterval) {
		this.costIntervals.add(costInterval);
		this.updateCost();
	}

	/**
	 * Removes a cost interval from this sampled waypoint.
	 * 
	 * @param costInterval the cost interval to be removed
	 */
	public void removeCostInterval(CostInterval costInterval) {
		this.costIntervals.remove(costInterval);
		this.updateCost();
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
	 * Updates the active cost of this sampled waypoint using its ato to search the
	 * costInterval tree.
	 */
	private void updateCost() {
		this.setCost(this.calculateCost(this.getAto()));
	}

	/**
	 * Calculates the cost of this waypoint at a given time by searching the
	 * costInterval tree.
	 * 
	 * @param start the time at this waypoint
	 * @return the cost of this waypoint at the given time
	 */
	public double calculateCost(ZonedDateTime start) {
		double cost = 1d; // simple cost of normalized distance

		Set<String> costIntervalIds = new HashSet<String>();
		// add all (weighted) cost of the cell
		List<Interval<ChronoZonedDateTime<?>>> intervals = this.getCostIntervals(start, start);
		for (Interval<ChronoZonedDateTime<?>> interval : intervals) {
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;

				// only add costs of different overlapping cost intervals
				if (!costIntervalIds.contains(costInterval.getId())) {
					costIntervalIds.add(costInterval.getId());

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

}
