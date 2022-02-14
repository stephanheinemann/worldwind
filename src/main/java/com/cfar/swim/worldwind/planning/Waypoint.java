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
import java.time.ZonedDateTime;

import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;
import com.cfar.swim.worldwind.util.Designatable;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Renderable;

/**
 * Realizes a waypoint of a trajectory featuring estimates for costs and time.
 * 
 * @author Stephan Heinemann
 *
 */
public class Waypoint extends Position
implements Cloneable, Comparable<Waypoint>, Depictable, Designatable {
	
	// TODO: possibly extend Waypoint with PrecisionWaypoint
	// TODO: use markers for actual track data (class Track)
	
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
	
	/** the designator of this waypoint */
	private String designator = "?";
	
	/** the precision position of this waypoint */
	private PrecisionPosition position = null;
	
	/** determines whether or not this waypoint is a POI */
	private boolean isPoi = false;
	
	/** the estimated cost of this waypoint in a trajectory */ 
	private double cost = Double.POSITIVE_INFINITY;
	
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
	 * Sets whether or not this waypoint is a POI.
	 * 
	 * @param isPoi true if this waypoint is a POI, false otherwise
	 */
	public void setPoi(boolean isPoi) {
		this.isPoi = isPoi;
	}
	
	/**
	 * Determines whether or not this waypoint is a POI.
	 * 
	 * @return true if this waypoint is a POI, false otherwise
	 */
	public boolean isPoi() {
		return this.isPoi;
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
	 * Gets the estimated cost of this waypoint in a trajectory.
	 * 
	 * @return the estimated cost of this wayppoint in a trajectory
	 */
	public double getCost() {
		return this.cost;
	}
	
	/**
	 * Sets the estimated cost of this waypoint in a trajectory.
	 * 
	 * @param cost the estimated cost of this waypoint in a trajectory
	 */
	public void setCost(double cost) {
		this.cost = cost;
	}
	
	/**
	 * Sets the estimated cost of this waypoint in a trajectory to infinity.
	 */
	public void setInfiniteCost() {
		this.cost = Double.POSITIVE_INFINITY;
	}
	
	/**
	 * Determines whether or not this waypoint has an infinite estimated cost.
	 * 
	 * @return true if this waypoint has an infinite estimated cost,
	 *         false otherwise
	 */
	public boolean hasInfiniteCost() {
		return (Double.POSITIVE_INFINITY == this.getCost());
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
	 * Determines whether or not this waypoint has a time to go.
	 * 
	 * @return true if this waypoint has a time to go, false otherwise
	 */
	public boolean hasTtg() {
		return (null != this.ttg);
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
	 * Determines whether or not this waypoint has a estimated time over.
	 * 
	 * @return true if this waypoint has a estimated time over, false otherwise
	 */
	public boolean hasEto() {
		return (null != this.eto);
	}
	
	// TODO: actuals belong to track points instead?
	
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
	 * Determines whether or not this waypoint has an actual time over.
	 * 
	 * @return true if this waypoint has an actual time over, false otherwise
	 */
	public boolean hasAto() {
		return (null != this.ato);
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
		if (null != this.depiction) {
			this.depiction.setDesignation(this.designator);
		}
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
	 * Clones this waypoint without its depiction.
	 * 
	 * @return the clone of this waypoint without its depiction 
	 * 
	 * @see Object#clone()
	 */
	@Override
	public Waypoint clone() {
		Waypoint waypoint = null;
		
		try {
			waypoint = (Waypoint) super.clone();
			waypoint.setDepiction(null);
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		
		return waypoint;
	}
	
	/**
	 * Compares this waypoint to another waypoint based on their actual time
	 * over (primary), estimated time over (secondary), position (tertiary).
	 * Note that this implementation is *not* consistent with equals and,
	 * therefore, sorted sets cannot be used with waypoints.
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
		
		// TODO: actuals versus estimates (waypoint versus trackpoint)
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
	 * Determines whether or not this waypoint equals another waypoint based on
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
	public final boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (o instanceof Waypoint)) {
			/*
			 * Avoid visiting existing positions although a 4D waypoint may
			 * very well revisit an existing position to avoid higher costs.
			 * The computed trajectories shall however be space-optimal but not
			 * necessarily time-optimal. To mitigate this issue, departure
			 * slots, or more generally, waypoint slots could be considered and
			 * take into account the aircraft capabilities appropriately
			 * (endurance). This could realize the concept of holding or
			 * loitering.
			 * 
			 * https://github.com/stephanheinemann/worldwind/issues/24
			 */
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
	public final int hashCode() {
		return this.position.hashCode();
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
