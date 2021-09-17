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
package com.cfar.swim.worldwind.planning;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;

import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.Movable;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.render.Renderable;

/**
 * Realizes a trajectory of waypoints which represents a path in both
 * space and time.
 * 
 * @author Stephan Heinemann
 *
 */
public class Trajectory extends Path implements Depictable {

	/** the depiction of this trajectory */
	private Depiction depiction = null;
	
	
	/**
	 * Constructs an empty trajectory.
	 * 
	 * @see Path#Path()
	 */
	public Trajectory() {
		super();
	}
	
	/**
	 * Constructs a trajectory between two specified waypoints.
	 * 
	 * @param waypointA the first waypoint
	 * @param waypointB the second waypoint
	 * 
	 * @see Path#Path(gov.nasa.worldwind.geom.Position, gov.nasa.worldwind.geom.Position)
	 */
	public Trajectory(Waypoint waypointA, Waypoint waypointB) {
		super(waypointA, waypointB);
	}
	
	/**
	 * Constructs a trajectory along a series of waypoints.
	 * 
	 * @param waypoints the waypoints
	 * 
	 * @see Path#Path(Iterable)
	 */
	public Trajectory(Iterable<? extends Waypoint> waypoints) {
		super(waypoints);
	}
	
	/**
	 * Determines whether or not this trajectory is empty (has no waypoints).
	 * 
	 * @return true if this trajectory is empty, false otherwise
	 */
	public boolean isEmpty() {
		return (0 == this.getWaypointsLength());
	}
	
	/**
	 * Gets the length of this trajectory in number of waypoints.
	 * 
	 * @return the length of this trajectory in number of waypoints
	 */
	public int getWaypointsLength() {
		return Iterables.size(this.getWaypoints());
	}
	
	/**
	 * Gets the POIs of this trajectory.
	 * 
	 * @return the POIs of this trajectory
	 */
	public List<Waypoint> getPois() {
		List<Waypoint> pois = new ArrayList<>();
		for (Waypoint waypoint : this.getWaypoints()) {
			if (waypoint.isPoi()) {
				pois.add(waypoint);
			}
		}
		
		return pois;
	}
	
	/**
	 * Gets a sub-trajectory of this trajectory from its start POI up to an end
	 * POI index.
	 * 
	 * @param poiIndex the end POI index
	 * 
	 * @return the sub-trajectory of this trajectory from its start POI up to
	 *         the end POI index
	 */
	public Trajectory getSubTrajectory(int poiIndex) {
		List<Waypoint> subWaypoints = new ArrayList<>();
		Iterator<? extends Waypoint> waypoints = this.getWaypoints().iterator();
		
		while ((0 <= poiIndex) && waypoints.hasNext()) {
			Waypoint waypoint = waypoints.next();
			subWaypoints.add(waypoint);
			if (waypoint.isPoi()) {
				poiIndex--;
			}
		}
		
		return new Trajectory(subWaypoints);
	}
	
	/**
	 * Gets the distance of this trajectory in meters.
	 * 
	 * @return the distance of this trajectory in meters
	 */
	public double getDistance() {
		double distance = 0d;
		
		for (Waypoint waypoint : this.getWaypoints()) {
			distance += waypoint.getDtg();
		}
		
		return distance;
	}
	
	/**
	 * Gets the estimated duration of this trajectory.
	 * 
	 * @return the estimated duration of this trajectory
	 */
	public Duration getDuration() {
		Duration duration = Duration.ZERO;
		
		for (Waypoint waypoint : this.getWaypoints()) {
			duration = duration.plus(waypoint.getTtg());
		}
		
		return duration;
	}
	
	/**
	 * Gets the estimated total cost of this trajectory.
	 * 
	 * @return the estimated total cost of this trajectory
	 */
	public double getCost() {
		return Iterables.getLast(this.getWaypoints()).getCost();
	}
	
	/**
	 * Gets the estimated risk (maximum cost) of this trajectory.
	 * 
	 * @return the estimated risk (maximum cost) of this trajectory
	 */
	public double getRisk() {
		double risk = 0d;
		
		for (Waypoint waypoint : this.getWaypoints()) {
			if (risk < waypoint.getCost()) {
				risk = waypoint.getCost();
			}
		}
		
		return risk;
	}
	
	/**
	 * Moves this trajectory by adding a specified position.
	 * 
	 * @param position the position to be added to this trajectory's position
	 * 
	 * @see Movable#move(Position)
	 */
	@Override
	public void move(Position position) {
		super.move(position);
		if (this.hasDepiction()) {
			this.depiction.move(position);
		}
	}
	
	/**
	 * Moves this trajectory to a specified position.
	 * 
	 * @param position the position to move this trajectory to
	 * 
	 * @see Movable#moveTo(Position)
	 */
	@Override
	public void moveTo(Position position) {
		super.moveTo(position);
		if (this.hasDepiction()) {
			this.depiction.moveTo(position);
		}
	}
	
	/**
	 * Gets the waypoints of this trajectory.
	 * 
	 * @return the waypoints of this trajectory
	 * 
	 * @see Path#getPositions()
	 */
	@SuppressWarnings("unchecked")
	public Iterable<? extends Waypoint> getWaypoints() {
		Iterable<Waypoint> waypoints = (Iterable<Waypoint>) super.getPositions();
		
		if (null == waypoints) {
			waypoints = Collections.emptySet();
		}
		
		return waypoints;
	}
	
	/**
	 * Gets the first waypoint of this trajectory.
	 * 
	 * @return the first waypoint of this trajectory if any, null otherwise
	 */
	public Waypoint getFirstWaypoint() {
		return Iterables.getFirst(this.getWaypoints(), null);
	}
	
	/**
	 * Gets the last waypoint of this trajectory.
	 * 
	 * @return the last waypoint of this trajectory if any, null otherwise
	 */
	public Waypoint getLastWaypoint() {
		return Iterables.getLast(this.getWaypoints(), null);
	}
	
	/**
	 * Gets the depiction of this trajectory.
	 * 
	 * @return the depiction of this trajectory
	 * 
	 * @see Depictable#getDepiction()
	 */
	@Override
	public Depiction getDepiction() {
		return this.depiction;
	}
	
	/**
	 * Sets the depiction of this trajectory.
	 * 
	 * @param depiction the depiction to be set
	 * 
	 * @see Depictable#setDepiction(Depiction)
	 */
	@Override
	public void setDepiction(Depiction depiction) {
		this.depiction = depiction;
	}
	
	/**
	 * Indicates whether or not this trajectory has a depiction.
	 * 
	 * @return true if this trajectory has a depiction, false otherwise
	 * 
	 * @see Depictable#hasDepiction()
	 */
	@Override
	public boolean hasDepiction() {
		return (null != this.depiction);
	}
	
	/**
	 * Renders this trajectory.
	 * 
	 * @param dc the drawing context
	 * 
	 * @see Renderable#render
	 */
	@Override
	public void render(DrawContext dc) {
		super.render(dc);
		if (null != this.depiction) {
			this.depiction.render(dc);
		}
		if (null != this.getWaypoints()) {
			for (Waypoint waypoint : this.getWaypoints()) {
				waypoint.render(dc);
			}
		}
	}
	
	/**
	 * Determines whether the waypoints of this trajectory equal the waypoints
	 * of another one.
	 * 
	 * @param o the other trajectory
	 * 
	 * @return true if the waypoints of this trajectory equal the waypoints of
	 *         the other trajectory, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;

		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			Trajectory ot = (Trajectory) o;
			equals = true;
			Iterator<? extends Waypoint> oti = ot.getWaypoints().iterator();
			Iterator<? extends Waypoint> tti = this.getWaypoints().iterator();
			while (equals && oti.hasNext() && tti.hasNext()) {
				if (!oti.next().equals(tti.next())) {
					equals = false;
				}
			}
			if (!equals || oti.hasNext() || tti.hasNext()) {
				equals = false;
			}
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this trajectory based on its waypoints.
	 * 
	 * @return the hash code of this trajectory based on its waypoints
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(this.getWaypoints());
	}
	
	/**
	 * Obtains the string representation of this trajectory.
	 * 
	 * @return the string representation of this trajectory
	 * 
	 * @see Object#toString()
	 */
	@Override
	public String toString() {
		String ts = "[ ";
		
		for (Waypoint waypoint : this.getWaypoints()) {
			ts += (waypoint.toString() + " ");
		}
		
		return (ts + "]");
	}
	
}
