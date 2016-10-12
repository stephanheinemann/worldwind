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
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;

/**
 * Describes an environment as a renderable on a globe (spatial aspect) with
 * associated cost intervals (temporal aspect).
 * 
 * @author Stephan Heinemann
 *
 */
public interface Environment extends TimedRenderable, ThresholdRenderable {
	
	/**
	 * Gets the globe of this environment.
	 * 
	 * @return the globe of this environment
	 */
	public Globe getGlobe();
	
	/**
	 * Sets the globe of this environment.
	 * 
	 * @param globe the globe of this environment
	 */
	public void setGlobe(Globe globe);
	
	// TODO: aggregate property time intervals for air density (temperature, pressure, humidity)
	
	/**
	 * Gets the center position of this environment in globe coordinates.
	 * 
	 * @return the center position of this environment in globe coordinates
	 */
	public Position getCenterPosition();
	
	/**
	 * Gets the neighbors of this environment.
	 * 
	 * @return the neighbors of this environment
	 */
	public Set<? extends Environment> getNeighbors();
	
	/**
	 * Indicates whether or not this environment is a neighbor of another
	 * environment.
	 * 
	 * @param neighbor the potential neighbor
	 * 
	 * @return true if this environment is a neighbor of the other environment,
	 *         false otherwise
	 */
	public boolean areNeighbors(Environment neighbor);
	
	/**
	 * Gets the neighbor positions of a position in this environment.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the neighbor positions of the position
	 */
	public Set<Position> getNeighbors(Position position);
	
	/**
	 * Indicates whether or not two positions are neighbors in this
	 * environment.
	 * 
	 * @param position the position
	 * @param neighbor the potential neighbor of the position
	 * 
	 * @return true if the two positions are neighbors, false otherwise
	 */
	public boolean areNeighbors(Position position, Position neighbor);
	
	/**
	 * Gets the children of this environment.
	 * 
	 * @return the children of this environment
	 */
	public Set<? extends Environment> getChildren();
	
	/**
	 * Gets the distance between two positions in this environment.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the distance between the two positions in this environment
	 */
	public double getDistance(Position position1, Position position2);
	
	/**
	 * Gets the normalized distance between two positions in this environment.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the normalized distance between the two positions in this
	 *         environment
	 */
	public double getNormalizedDistance(Position position1, Position position2);
	
	/**
	 * Adds a cost interval to this environment.
	 * 
	 * @param costInterval the cost interval to be added to this environment
	 */
	public void addCostInterval(CostInterval costInterval);
	
	/**
	 * Removes a cost interval from this environment.
	 * 
	 * @param costInterval the cost interval to be removed from this environment
	 */
	public void removeCostInterval(CostInterval costInterval);
	
	/**
	 * Gets all (overlapping) cost intervals at a specified time instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return the cost intervals at the specified time instant
	 */
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time);
	
	/**
	 * Gets all (overlapping) cost intervals within a specified time span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the cost intervals within the specified time span
	 */
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end);

	/**
	 * Gets the accumulated cost of this environment within a specified time
	 * span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the accumulated cost of this environment within a the specified
	 *         time span
	 */
	public double getCost(ZonedDateTime start, ZonedDateTime end);
	
	/**
	 * Gets the step cost from a position to its neighbor position of this
	 * environment between a start and an end time given a cost policy and
	 * risk policy.
	 * 
	 * @param position the position
	 * @param neighbor its neighbor position
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the step cost from a position to its neighbor position
	 */
	public double getStepCost(
			Position position, Position neighbor,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy);
	
	/**
	 * Gets the step cost from the center of this environment to the center of
	 * a neighboring environment between a start and an end time given a cost
	 * policy and risk policy.
	 * 
	 * @param neighbor the neighboring environment
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the step cost from the center of this environment to the center
	 *         of the neighboring environment
	 */
	public double getStepCost(
			Environment neighbor,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy);
	
}
