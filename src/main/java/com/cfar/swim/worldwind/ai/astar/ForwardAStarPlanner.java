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
package com.cfar.swim.worldwind.ai.astar;

import java.time.ZonedDateTime;
import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PositionEstimate;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;

public class ForwardAStarPlanner extends AbstractPlanner {

	PriorityQueue<PositionEstimate> open = new PriorityQueue<PositionEstimate>();
	Set<PositionEstimate> closed = new HashSet<PositionEstimate>();
	PositionEstimate start = null;
	PositionEstimate goal = null;
	ZonedDateTime etd = null;
	
	public ForwardAStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	protected Path getPath(PositionEstimate positionEstimate) {
		ArrayDeque<Position> positions = new ArrayDeque<Position>(); 
		
		while ((null != positionEstimate) && (positionEstimate != positionEstimate.getParent())) {
			positions.addFirst(positionEstimate.getPosition());
			positionEstimate = positionEstimate.getParent();
		}
		
		return new Path(positions);
	}
	
	protected void updatePositionEstimate(PositionEstimate source, PositionEstimate target) {
		double gOld = target.getG();
		this.computeCost(source, target);
		if (target.getG() < gOld) {
			if (!this.open.contains(target)) {
				target.setH(this.getEnvironment().getNormalizedDistance(
						target.getPosition(), this.goal.getPosition()));
				this.open.add(target);
			}
		}
	}
	
	protected void computeCost(PositionEstimate source, PositionEstimate target) {
		
	}
	
	@Override
	public Path plan(Position origin, Position destination, ZonedDateTime etd) {
		this.start = new PositionEstimate(origin);
		this.start.setG(0);
		this.start.setH(this.getEnvironment().getNormalizedDistance(origin, destination));
		this.start.setParent(this.start);
		this.start.setEto(etd);
		
		this.goal = new PositionEstimate(destination);
		this.goal.setH(0);
		
		this.etd = etd;
		
		while (null != this.open.peek()) {
			PositionEstimate source = this.open.poll();
			if (source.equals(this.goal)) {
				return this.getPath(source);
			}
			this.closed.add(source);
			Set<Position> neighbors = this.getEnvironment().getNeighbors(source.getPosition());
			for (Position neighbor : neighbors) {
				PositionEstimate target = new PositionEstimate(neighbor);
				if (!closed.contains(target)) {
					if (open.contains(target)) {
						PositionEstimate o = open.stream().filter(s -> s.equals(target)).findFirst().get();
						this.updatePositionEstimate(source, o);
					} else {
						this.updatePositionEstimate(source, target);
					}
				}
			}
		}
		
		return null;
	}

	@Override
	public Path plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		// TODO Auto-generated method stub
		return null;
	}

}
