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

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an edge of a roadmap based on two sampled positions.
 * 
 * @author Henrique Ferreira
 * @author Manuel Rosa
 *
 */
public class Edge {

	// TODO: Should we store an edge cost?
	// It would be beneficial to not call calculate cost every time and useful if
	// drawing edges is desired

	/** the first position of this edge */
	private Position position1;

	/** the second position of this edge */
	private Position position2;

	/** the geometric line between the two positions */
	private Line line;

	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(
			CostInterval.comparator);

	/**
	 * Constructs an Edge based on two positions.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 */
	public Edge(Position position1, Position position2) {
		this.position1 = position1;
		this.position2 = position2;
	}

	/**
	 * Constructs an Edge based on two positions and a line.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * @param line the line formed by the two positions
	 */
	public Edge(Position position1, Position position2, Line line) {
		this.position1 = position1;
		this.position2 = position2;
		this.line = line;
	}

	/**
	 * Gets the first position of this edge.
	 * 
	 * @return the first position of this edge
	 */
	public Position getPosition1() {
		return position1;
	}

	/**
	 * Sets the first position of this edge.
	 * 
	 * @param position1 the first position of this edge
	 */
	public void setPosition1(Position position1) {
		this.position1 = position1;
	}

	/**
	 * Gets the second position of this edge.
	 * 
	 * @return the second position of this edge
	 */
	public Position getPosition2() {
		return position2;
	}

	/**
	 * Sets the second position of this edge.
	 * 
	 * @param position1 the second position of this edge
	 */
	public void setPosition2(Position position2) {
		this.position2 = position2;
	}

	/**
	 * Receives one position and gets the other position of this edge.
	 * 
	 * @param position the position to be checked
	 * 
	 * @return the other position of this edge
	 */
	public Position getOtherPosition(Position position) {
		if (position.equals(position1)) {
			return position2;
		} else if (position.equals(position2)) {
			return position1;
		} else {
			return null;
		}
	}

	/**
	 * Sets the line between the two positions in this edge.
	 * 
	 * @return the line between positions
	 */
	public Line getLine() {
		return line;
	}

	/**
	 * Gets the line between the two positions in this edge.
	 * 
	 * @param line the line between positions to set
	 */
	public void setLine(Line line) {
		this.line = line;
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
	 * Adds a cost interval to the cost interval tree of this edge.
	 * 
	 * @param costInterval the cost interval to be added
	 */
	public void addCostInterval(CostInterval costInterval) {
		this.costIntervals.add(costInterval);
	}

	/**
	 * Removes a cost interval from the cost interval tree of this edge.
	 * 
	 * @param costInterval the cost interval to be removed
	 */
	public void removeCostInterval(CostInterval costInterval) {
		this.costIntervals.remove(costInterval);
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
	 * Calculates the cost of this edge within a specified time span by searching
	 * the cost interval tree.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the accumulated cost of this edge within the specified time span
	 */
	public double calculateCostOld(ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy) {
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
	public double calculateCost(ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy, RiskPolicy riskPolicy) {
		double cost = 1d; // simple cost of normalized distance

		Set<ZonedDateTime> times = new HashSet<ZonedDateTime>();
		times.add(start);
		times.add(end);

		Set<String> costIntervalIds = new HashSet<String>();

		// add all (weighted) cost of the cell
		List<Interval<ChronoZonedDateTime<?>>> intervals = this.getCostIntervals(start, end);

		for (Interval<ChronoZonedDateTime<?>> interval : intervals) {
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;

				// only consider different overlapping cost intervals
				if (!costIntervalIds.contains(costInterval.getId())) {
					costIntervalIds.add(costInterval.getId());

					// Add lower limit to times if it is higher than start
					if (costInterval.getLower().compareTo(start) > 0)
						times.add(costInterval.getLower());
					// Add upper limit to times if it is lower than end
					if (costInterval.getUpper().compareTo(end) < 0)
						times.add(costInterval.getUpper());
				}
			}
		}

		List<ZonedDateTime> sortedTimes = times.stream().sorted().collect(Collectors.toList());
		Iterator<ZonedDateTime> iterator = sortedTimes.iterator();
		List<Double> costListTemporal = new ArrayList<Double>();
		ZonedDateTime startPart = iterator.next(), endPart;
		double costPart = 1d, costAux = 0d;

		// For every sub-Interval
		while (iterator.hasNext()) {
			endPart = iterator.next();
			costPart = 1d;
			for (Interval<ChronoZonedDateTime<?>> interval : intervals) {
				if (interval instanceof CostInterval) {
					CostInterval costInterval = (CostInterval) interval;

					// Only consider intervals that fully contain this sub-interval
					if (costInterval.getLower().compareTo(startPart) < 0
							&& costInterval.getUpper().compareTo(endPart) > 0) {
						if ((interval instanceof WeightedCostInterval)) {
							costAux += ((WeightedCostInterval) interval).getWeightedCost();
						} else {
							costAux += costInterval.getCost();
						}
					}
				}
			}
			costPart = costAux == 0d ? costPart : costAux;
			costPart = riskPolicy.satisfies(costPart - 1) ? costPart : Double.POSITIVE_INFINITY;
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
			// TODO: Replace by weighted average with duration of time interval
			cost = costListTemporal.stream().mapToDouble(Double::doubleValue).max().getAsDouble();
			break;
		}
		System.out.println(costPolicy + " " + riskPolicy + "\tcost = " + cost);

		return cost;
	}

	/**
	 * Checks whether or not the given position is part of this edge.
	 * 
	 * @param position the position to be checked
	 * 
	 * @return true if the position is part of this edge, false otherwise
	 */
	public boolean contains(Position position) {
		if (position.equals(position1) || position.equals(position2))
			return true;
		else
			return false;
	}

	/**
	 * Gets the hash code of this edge based on the hash code of the two positions.
	 * 
	 * @return the multiplication of the hash code of the two positions
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		int result;
		result = position1.hashCode();
		result = result * position2.hashCode();
		return result;
	}

	/**
	 * Indicates whether or not this edge equals another edge based on their
	 * positions.
	 * 
	 * @param obj the other edge
	 * 
	 * @return true, if the positions are the same as the positions of the other
	 *         edge (regardless of order), false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object obj) {
		boolean equals = false;

		if (obj instanceof Edge) {
			equals = this.position1.equals(((Edge) obj).position1) && this.position2.equals(((Edge) obj).position2)
					|| this.position1.equals(((Edge) obj).position2) && this.position2.equals(((Edge) obj).position1);
		}

		return equals;
	}
}
