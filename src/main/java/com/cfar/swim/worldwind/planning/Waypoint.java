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

import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a waypoint of a trajectory featuring estimates for costs and time.
 * 
 * @author Stephan Heinemann
 *
 */
public class Waypoint extends Position implements Comparable<Waypoint> {
	
	// TODO: possibly extend Waypoint with PrecisionWaypoint
	
	/** the parent waypoint of this waypoint in a trajectory */
	private Waypoint parent = null;
	
	/** the precision position of this waypoint */
	private PrecisionPosition position = null;
	
	/** the estimated current cost of this waypoint */
	private double g = Double.POSITIVE_INFINITY;
	
	// TODO: (position, estimated cost tuple: (g1, g2)) for more advanced versions
	
	/** the estimated remaining cost of this waypoint */
	private double h = Double.POSITIVE_INFINITY;
	
	/** the estimated time at this waypoint */
	private ZonedDateTime eto = null;
	
	// TODO: include actual time over and correct resulting cruise performance
	
	/**
	 * Constructs a waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 */
	public Waypoint(Position position) {
		super(position.getLatitude(), position.getLongitude(), position.getElevation());
		this.position = new PrecisionPosition(position);
	}
	
	/**
	 * Gets the estimated current cost of this waypoint.
	 * 
	 * @return the estimated current cost of this waypoint
	 */
	public double getG() {
		return g;
	}
	
	/**
	 * Sets the estimated current cost of this waypoint.
	 * 
	 * @param g the estimated current cost of this waypoint
	 */
	public void setG(double g) {
		this.g = g;
	}
	
	/**
	 * Gets the estimated remaining cost of this waypoint.
	 * 
	 * @return the estimated remaining cost of this waypoint
	 */
	public double getH() {
		return h;
	}
	
	/**
	 * Sets the estimated remaining cost of this waypoint.
	 * 
	 * @param h the estimated remaining cost of this waypoint
	 */
	public void setH(double h) {
		this.h = h;
	}
	
	/**
	 * Gets the estimated total cost of this waypoint.
	 * 
	 * @return the estimated total cost of this waypoint
	 */
	public double getF() {
		return this.g + this.h;
	}
	
	/**
	 * Gets the parent waypoint of this waypoint.
	 * 
	 * @return the parent waypoint of this waypoint
	 */
	public Waypoint getParent() {
		return parent;
	}
	
	/**
	 * Sets the parent waypoint of this waypoint.
	 * 
	 * @param parent the parent waypoint of this waypoint
	 */
	public void setParent(Waypoint parent) {
		this.parent = parent;
	}
	
	/**
	 * Gets the precision position of this waypoint.
	 * 
	 * @return the precision position of this waypoint
	 */
	public PrecisionPosition getPrecisionPosition() {
		return this.position;
	}
	
	/**
	 * Compares this waypoint to another waypoint based on their estimated
	 * total costs. It the estimated total costs of both waypoints is equal,
	 * then break ties in favor of higher estimated current costs.
	 * 
	 * @param o the other waypoint
	 * 
	 * @return -1, 0, 1, if this waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their total
	 *         estimated cost
	 * 
	 * @see Comparable#compareTo(Object)
	 */
	@Override
	public int compareTo(Waypoint o) {
		int compareTo = new Double(this.getF()).compareTo(o.getF());
		if (0 == compareTo) {
			// break ties in favor of higher G-values
			compareTo = new Double(o.getG()).compareTo(this.getG());
		}
		
		return compareTo;
	}
	
	/**
	 * Indicates whether or not this waypoint equals another waypoint based on
	 * their precision position.
	 * 
	 * @param o the other waypoint
	 * 
	 * @return true, if the precision position of this waypoint equals the
	 *         precision position of the other waypoint, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (o instanceof Waypoint) {
			equals = this.position.equals(((Waypoint) o).position);
		}
	
		return equals;
	}
	
	/**
	 * Gets the hash code of this waypoint based on its precision position.
	 * 
	 * @return the hash code of this waypoint based on its precision position
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return this.position.hashCode();
	}
	
	/**
	 * Gets the estimated time at this waypoint.
	 * 
	 * @return the estimated time at this waypoint
	 */
	public ZonedDateTime getEto() {
		return eto;
	}
	
	/**
	 * Sets the estimated time at this waypoint.
	 * 
	 * @param eto the estimated time at this waypoint
	 */
	public void setEto(ZonedDateTime eto) {
		this.eto = eto;
	}
	
}