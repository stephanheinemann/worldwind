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
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PositionEstimate;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a basic forward A* planner that plans a trajectory of an aircraft
 * in an environment considering a local cost and risk policy.
 * 
 * @author Stephan Heinemann
 *
 */
public class ForwardAStarPlanner extends AbstractPlanner {

	// TODO: plan Trajectory extends Path, aggregates Waypoint
	// TODO: PositionEstimate -> Waypoint extends Position
	
	/** the priority queue of expandable waypoints */
	private PriorityQueue<PositionEstimate> open = new PriorityQueue<PositionEstimate>();
	
	/** the set of expanded waypoints */
	private Set<PositionEstimate> closed = new HashSet<PositionEstimate>();
	
	/** the start waypoint */
	private PositionEstimate start = null;
	
	/** the goal waypoint */
	private PositionEstimate goal = null;
	
	private ArrayDeque<PositionEstimate> plan = new ArrayDeque<PositionEstimate>();
	
	public ForwardAStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	public ArrayDeque<PositionEstimate> getPlan() {
		return plan; // TODO: remove, return Trajectory instead
	}
	
	protected Path computePath(PositionEstimate positionEstimate) {
		ArrayDeque<Position> positions = new ArrayDeque<Position>(); 
		
		while ((null != positionEstimate)) {
			positions.addFirst(positionEstimate.getPosition());
			plan.addFirst(positionEstimate);
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
			} else {
				// priority queue requires re-insertion of modified object
				this.open.remove(target);
				this.open.add(target);
			}
		}
	}
	
	protected void computeCost(PositionEstimate source, PositionEstimate target) {
		Path leg = new Path(source.getPosition(), target.getPosition());
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		
		double cost = this.getEnvironment().getStepCost(
				source.getPosition(), target.getPosition(),
				source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy());
		
		if ((source.getG() + cost) < target.getG()) {
			target.setParent(source);
			target.setG(source.getG() + cost);
			target.setEto(end);
		}
	}
	
	@Override
	public Path plan(Position origin, Position destination, ZonedDateTime etd) {
		this.open.clear();
		this.closed.clear();
		this.plan.clear();
		
		this.start = new PositionEstimate(origin);
		this.start.setG(0);
		this.start.setH(this.getEnvironment().getNormalizedDistance(origin, destination));
		this.start.setEto(etd);
		
		this.goal = new PositionEstimate(destination);
		this.goal.setH(0);
		// if a goal is not a waypoint in the environment, then its
		// goal region has to be determined for the final expansion
		Set<PrecisionPosition> goalRegion = this.getEnvironment().getAdjacentWaypoints(destination)
				.stream()
				.map(PrecisionPosition::new)
				.collect(Collectors.toSet());
		
		this.open.add(this.start);
		
		while (null != this.open.peek()) {
			PositionEstimate source = this.open.poll();
			if (source.equals(this.goal)) {
				return this.computePath(source);
			}
			this.closed.add(source);
			
			Set<Position> neighbors = this.getEnvironment().getNeighbors(source.getPosition());
			// if a start has no neighbors, then it is not a waypoint in the
			// environment and its adjacent waypoints have to be determined for
			// initial expansion
			if (neighbors.isEmpty()) {
				neighbors = this.getEnvironment().getAdjacentWaypoints(source.getPosition());
			}
			// expand a goal region position towards the goal
			if (goalRegion.contains(new PrecisionPosition(source.getPosition()))) {
				neighbors.add(destination);
			}
			
			for (Position neighbor : neighbors) {
				PositionEstimate target = new PositionEstimate(neighbor);
				if (!closed.contains(target)) {
					if (open.contains(target)) {
						PositionEstimate visited = open.stream().filter(s -> s.equals(target)).findFirst().get();
						this.updatePositionEstimate(source, visited);
					} else {
						this.updatePositionEstimate(source, target);
					}
				}
			}
		}
		
		return new Path();
	}

	@Override
	public Path plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		List<Position> positions = new ArrayList<Position>();
		ArrayDeque<PositionEstimate> plan = new ArrayDeque<PositionEstimate>();
		Position currentOrigin = origin;
		ZonedDateTime currentEtd = etd;
		Iterator<Position> waypointIterator = waypoints.iterator();
		
		while (waypointIterator.hasNext()) {
			Position currentDestination = waypointIterator.next();
			
			if (!(new PrecisionPosition(currentOrigin).equals(new PrecisionPosition(currentDestination)))) {
				this.plan(currentOrigin, currentDestination, currentEtd);
				
				ArrayDeque<PositionEstimate> legPlan = this.getPlan();
				if ((!plan.isEmpty()) && (!legPlan.isEmpty())) {
					legPlan.poll();
					legPlan.getFirst().setParent(plan.getLast());
				}
				
				plan.addAll(legPlan);
				PositionEstimate positionEstimate = null;
				
				while (null != legPlan.peek()) {
					positionEstimate = legPlan.poll();
					positions.add(positionEstimate.getPosition());
				}
				
				if (null != positionEstimate) {
					currentOrigin = positionEstimate.getPosition();
					currentEtd = positionEstimate.getEto();
				} else {
					positions.clear();
					return new Path(positions);
				}
			}
		}
		
		this.plan = plan;
		return new Path(positions);
	}

}
