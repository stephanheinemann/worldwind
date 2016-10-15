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
 * Realizes an estimate for a planned position.
 * 
 * @author Stephan Heinemann
 *
 */
public class PositionEstimate implements Comparable<PositionEstimate> {
	
	// TODO: PositionEstimate -> Waypoint (4D) extends Position (3D)
	// TODO: Trajectory (4D) extends Path (3D) aggregates Waypoint
	
	/** the parent position estimate of this position estimate */
	private PositionEstimate parent = null;
	
	/** the position of this position estimate */
	private PrecisionPosition position = null;
	
	/** the estimated current cost of this position estimate */
	private double g = Double.POSITIVE_INFINITY;
	
	// TODO: (position, estimated cost tuple: (g1, g2)) for more advanced versions
	
	/** the estimated remaining cost of this position estimate */
	private double h = Double.POSITIVE_INFINITY;
	
	/** the estimated time over the position of this position estimate */
	private ZonedDateTime eto = null;
	
	// TODO: include actual time over and correct resulting cruise performance
	
	/**
	 * Constructs a position estimate at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 */
	public PositionEstimate(Position position) {
		this.position = new PrecisionPosition(position);
	}
	
	/**
	 * Gets the estimated current cost of this position estimate.
	 * 
	 * @return the estimated current cost of this position estimate
	 */
	public double getG() {
		return g;
	}
	
	/**
	 * Sets the estimated current cost of this position estimate.
	 * 
	 * @param g the estimated current cost of this position estimate
	 */
	public void setG(double g) {
		this.g = g;
	}
	
	/**
	 * Gets the estimated remaining cost of this position estimate.
	 * 
	 * @return the estimated remaining cost of this position estimate
	 */
	public double getH() {
		return h;
	}
	
	/**
	 * Sets the estimated remaining cost of this position estimate.
	 * 
	 * @param h the estimated remaining cost of this position estimate
	 */
	public void setH(double h) {
		this.h = h;
	}
	
	/**
	 * Gets the estimated total cost of this position estimate.
	 * 
	 * @return the estimated total cost of this position estimate
	 */
	public double getF() {
		return this.g + this.h;
	}
	
	/**
	 * Gets the parent position estimate of this position estimate.
	 * 
	 * @return the parent position estimate of this position estimate
	 */
	public PositionEstimate getParent() {
		return parent;
	}
	
	/**
	 * Sets the parent position estimate of this position estimate.
	 * 
	 * @param parent the parent position estimate of this position estimate
	 */
	public void setParent(PositionEstimate parent) {
		this.parent = parent;
	}
	
	/**
	 * Gets the position of this position estimate.
	 * 
	 * @return the position of this position estimate
	 */
	public Position getPosition() {
		return this.position.getOriginal();
	}
	
	/**
	 * Compares this position estimate to another position estimate
	 * based on their total estimated cost.
	 * 
	 * @param o the other position estimate
	 * 
	 * @return -1, 0, 1, if this position estimate is less than, equal, or
	 *         greater, respectively, than the other position estimate based
	 *         on their total estimated cost
	 * 
	 * @see Comparable#compareTo(Object)
	 */
	@Override
	public int compareTo(PositionEstimate o) {
		return new Double(this.getF()).compareTo(o.getF());
	}
	
	/**
	 * Indicates whether or not this position estimate equals another position
	 * estimate based on their position.
	 * 
	 * @param o the other position estimate
	 * 
	 * @return true, if the position of this position estimate equals the
	 *         position of the other position estimate, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (o instanceof PositionEstimate) {
			equals = this.position.equals(((PositionEstimate) o).position);
		}
	
		return equals;
	}
	
	/**
	 * Gets the hash code of this position estimate based on its position.
	 * 
	 * @return the hash code of this position estimate based on its position
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return this.position.hashCode();
	}
	
	/**
	 * Gets the estimated time over the position of this position estimate.
	 * 
	 * @return the estimated time over the position of this position estimate
	 */
	public ZonedDateTime getEto() {
		return eto;
	}
	
	/**
	 * Sets the estimate time over the position of this position estimate.
	 * 
	 * @param eto the estimate time over the position of this position estimate
	 */
	public void setEto(ZonedDateTime eto) {
		this.eto = eto;
	}
	
}
