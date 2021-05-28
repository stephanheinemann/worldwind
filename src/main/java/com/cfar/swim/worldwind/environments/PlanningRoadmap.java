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

import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.render.Obstacle;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;

// TODO: a planning roadmap could be fed by AIXM data to establish valid VFR/IFR routes
// TODO: there could be a geometric roadmap base class (similar to planning grid)
public class PlanningRoadmap implements DynamicEnvironment, MultiResolutionEnvironment, StructuredEnvironment {

	@Override
	public ZonedDateTime getTime() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setTime(ZonedDateTime time) {
		// TODO Auto-generated method stub

	}

	@Override
	public void render(DrawContext dc) {
		// TODO Auto-generated method stub

	}

	@Override
	public void setThreshold(double threshold) {
		// TODO Auto-generated method stub

	}

	@Override
	public double getThreshold() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public Globe getGlobe() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setGlobe(Globe globe) {
		// TODO Auto-generated method stub

	}

	@Override
	public Position getCenterPosition() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Set<Position> getNeighbors(Position position) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean areNeighbors(Position position, Position neighbor) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public double getDistance(Position position1, Position position2) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		// TODO Auto-generated method stub
		return 0;
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
	public boolean isWaypointPosition(Position position) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Set<Position> getAdjacentWaypointPositions(Position position) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isAdjacentWaypointPosition(Position position, Position waypoint) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean contains(Position position) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public boolean isRefined() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void refine(int density) {
		// TODO Auto-generated method stub	
	}
	
	public void coarsen() {
	}

	@Override
	public boolean embed(Obstacle obstacle) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean unembed(Obstacle obstacle) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void refresh(Obstacle obstacle) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void unembedAll() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public Set<Position> getAffectedWaypointPositions(Obstacle obstacle) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Set<Position> getAffectedWaypointPositions(Set<Obstacle> obstacles) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double getBaseCost() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void addStructuralChangeListener(StructuralChangeListener listener) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void removeStructuralChangeListener(StructuralChangeListener listener) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void notifyStructuralChangeListeners() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean hasGlobe() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isInsideGlobe(Position position) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public double toDistance(double normalizedDistance) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double toNormalizedDistance(double distance) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getCost(ZonedDateTime time) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getResolution() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void setResolution(double resolution) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void coarsen(int factor) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public double getNormalizer() {
		// TODO Auto-generated method stub
		return 0;
	}

}
