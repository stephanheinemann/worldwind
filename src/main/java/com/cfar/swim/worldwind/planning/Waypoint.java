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
import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;
import com.cfar.swim.worldwind.util.Designatable;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Renderable;
import java.time.Duration;

/**
 * Realizes a waypoint of a trajectory featuring estimates for costs and time.
 * 
 * @author Stephan Heinemann
 *
 */
public class Waypoint extends Position implements Comparable<Waypoint>, Depictable, Designatable {
	
	/** the symbol identification code for an action waypoint */
	public static final String SICD_NAV_WAYPOINT_ACTION = "GFGPGPPW------X"; // G*GPGPPW--****X
	
	/** the symbol identification code for a reference waypoint */
	public static final String SIDC_NAV_WAYPOINT_REFERENCE = "GFGPGPRW------X"; // G*GPGPRW--****X
	
	/** the symbol identification code for a corridor waypoint */
	public static final String SIDC_NAV_WAYPOINT_CORRIDOR = "GFGPGPRC------X"; // G*GPGPRC--****X
	
	/** the symbol identification code for a point of interest */
	public static final String SIDC_NAV_WAYPOINT_POI = "GFGPGPRI------X"; //G*GPGPRI--****X
	
	/** the symbol identification code for a route waypoint */
	public static final String SICD_NAV_WAYPOINT_ROUTE = "GFGPGPOW------X"; // G*GPGPO---****X, G*GPGPOW--****X
	
	/** the symbol identification code for a route rendezvous waypoint */
	public static final String SICD_NAV_WAYPOINT_ROUTE_RENDEZVOUS = "GFGPGPOZ------X"; // G*GPGPOZ--****X
	
	/** the symbol identification code for a route diversion waypoint */
	public static final String SICD_NAV_WAYPOINT_ROUTE_DIVERSION = "GFGPGPOD------X"; // G*GPGPOD--****X
	
	/** the symbol identification code for a route point of intended movement */
	public static final String SICD_NAV_WAYPOINT_ROUTE_PIM = "GFGPGPOP------X"; // G*GPGPOP--****X
	
	/** the symbol identification code for a route reference waypoint */
	public static final String SICD_NAV_WAYPOINT_ROUTE_REFERENCE = "GFGPGPOR------X"; // G*GPGPOR--****X
	
	// TODO: possibly extend Waypoint with PrecisionWaypoint
	// TODO: use markers for actual track data (class Track)
	
	/** the designator of this waypoint */
	private String designator = "?";
	
	/** the parent waypoint of this waypoint in a trajectory */
	private Waypoint parent = null;
	
	/** the precision position of this waypoint */
	private PrecisionPosition position = null;
	
	/** the distance to go from this waypoint (to the next one) */
	private double dtg = Double.POSITIVE_INFINITY;
	
	/** the time to go from this waypoint (to the next one) */
	private Duration ttg = null;
	
	/** the estimated time at this waypoint */
	private ZonedDateTime eto = null;
	
	/** the actual time at this waypoint */
	private ZonedDateTime ato = null;
	
	// TODO: include actual time over and correct resulting cruise performance
	
	/** the depiction of this waypoint */
	private Depiction depiction = null;
	
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
	 * Gets the designator of this waypoint.
	 * 
	 * @return the designator of this waypoint
	 * 
	 * @see Designatable#getDesignator()
	 */
	@Override
	public String getDesignator() {
		return this.designator;
	}
	
	/**
	 * Sets the designator of this waypoint.
	 * 
	 * @param designator the designator of this waypoint
	 * 
	 * @see Designatable#setDesignator(String)
	 */
	@Override
	public void setDesignator(String designator) {
		this.designator = designator;
		
		if (null != this.depiction) {
			this.depiction.setDesignation(this.designator);
		}
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
	 * Compares this waypoint to another waypoint based on their actual time
	 * over (primary), estimated time over (secondary), position (tertiary).
	 * 
	 * @param waypoint the other waypoint
	 * 
	 * @return -1, 0, 1, if this waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their actual time
	 *         over (primary), estimated time over (secondary), position
	 *         (tertiary)
	 * 
	 * @see Comparable#compareTo(Object)
	 */
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;
		
		if ((null != this.ato) && (null != waypoint.ato)) {
			compareTo = this.ato.compareTo(waypoint.ato);
		} else if ((null != this.eto) && (null != waypoint.eto)) {
			compareTo = this.eto.compareTo(waypoint.eto);
		} else {
			compareTo = this.position.compareTo(waypoint);
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
	 * Gets the distance to go from this waypoint (to the next one).
	 * 
	 * @return the distance to go from this waypoint in meters
	 */
	public double getDtg() {
		return this.dtg;
	}
	
	/**
	 * Sets the distance to go from this waypoint (to the next one).
	 * 
	 * @param dtg the distance to go from this waypoint in meters
	 */
	public void setDtg(double dtg) {
		this.dtg = dtg;
	}
	
	/**
	 * Gets the time to go from this waypoint (to the next one).
	 * 
	 * @return the time to go from this waypoint
	 */
	public Duration getTtg() {
		return this.ttg;
	}
	
	/**
	 * Sets the time to go from this waypoint (to the next one).
	 * 
	 * @param ttg the time to go from this waypoint
	 */
	public void setTtg(Duration ttg) {
		this.ttg = ttg;
	}
	
	/**
	 * Gets the estimated time at this waypoint.
	 * 
	 * @return the estimated time at this waypoint
	 */
	public ZonedDateTime getEto() {
		return this.eto;
	}
	
	/**
	 * Sets the estimated time at this waypoint.
	 * 
	 * @param eto the estimated time at this waypoint
	 */
	public void setEto(ZonedDateTime eto) {
		this.eto = eto;
	}
	
	/**
	 * Gets the actual time at this waypoint.
	 * 
	 * @return the actual time at this waypoint
	 */
	public ZonedDateTime getAto() {
		return this.ato;
	}
	
	/**
	 * Sets the actual time at this waypoint.
	 * 
	 * @param ato the actual time at this waypoint
	 */
	public void setAto(ZonedDateTime ato) {
		this.ato = ato;
	}
	
	/**
	 * Gets the depiction of this waypoint.
	 * 
	 * @return the depiction of this waypoint
	 * 
	 * @see Depictable#getDepiction()
	 */
	@Override
	public Depiction getDepiction() {
		return this.depiction;
	}
	
	/**
	 * Sets the depiction of this waypoint.
	 * 
	 * @param depiction the depiction of this waypoint
	 * 
	 * @see Depictable#setDepiction(Depiction)
	 */
	@Override
	public void setDepiction(Depiction depiction) {
		this.depiction = depiction;
		this.depiction.setDesignation(this.designator);
	}
	
	/**
	 * Indicates whether or not this waypoint has a depiction.
	 * 
	 * @return true if this waypoint has a depiction, false otherwise
	 * 
	 * @see Depictable#hasDepiction()
	 */
	@Override
	public boolean hasDepiction() {
		return (null != this.depiction);
	}
	
	/**
	 * Renders this waypoint.
	 * 
	 * @see Renderable#render(DrawContext)
	 */
	@Override
	public void render(DrawContext dc) {
		if (null != this.depiction) {
			this.depiction.render(dc);
		}
	}
	
}
