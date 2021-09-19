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
package com.cfar.swim.worldwind.planners.managed;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes the managed goals of a managed planner.
 * 
 * @author Stephan Heinemann
 *
 */
public class ManagedGoals {
	
	/** the origin of these managed goals */
	private Position origin = Position.ZERO;
	
	/** the destination of these managed goals */
	private Position destination = Position.ZERO;
	
	/** the points of interest of these managed goals */
	private List<Position> pois = new ArrayList<>();
	
	/** the estimated time of departure of these managed goals */
	private ZonedDateTime etd = ZonedDateTime.now();
	
	/**
	 * Gets the origin of these managed goals.
	 * 
	 * @return the origin of these managed goals
	 */
	public Position getOrigin() {
		return this.origin;
	}
	
	/**
	 * Sets the origin of these managed goals.
	 * 
	 * @param origin the origin to be set
	 * 
	 * @throws IllegalArgumentException if the origin is null
	 */
	public void setOrigin(Position origin) {
		if (null == origin) {
			throw new IllegalArgumentException();
		}
		this.origin = origin;
	}
	
	/**
	 * Gets the destination of these managed goals.
	 * 
	 * @return the destination of these managed goals
	 */
	public Position getDestination() {
		return this.destination;
	}
	
	/**
	 * Sets the destination of these managed goals.
	 * 
	 * @param destination the destination to be set
	 * 
	 * @throws IllegalArgumentException if the destination is null
	 */
	public void setDestination(Position destination) {
		if (null == destination) {
			throw new IllegalArgumentException();
		}
		this.destination = destination;
	}

	/**
	 * Gets the estimated time of departure of these managed goals.
	 * 
	 * @return the estimated time of departure of these managed goals
	 */
	public ZonedDateTime getEtd() {
		return this.etd;
	}

	/**
	 * Sets the estimated time of departure of these managed goals.
	 * 
	 * @param etd the estimated time of departure to be set
	 * 
	 * @throws IllegalArgumentException if the estimated time of departure is
	 *                                  null
	 */
	public void setEtd(ZonedDateTime etd) {
		if (null == etd) {
			throw new IllegalArgumentException();
		}
		this.etd = etd;
	}
	
	/**
	 * Gets the points of interest of these managed goals.
	 * 
	 * @return the points of interest of these managed goals
	 */
	public List<Position> getPois() {
		return Collections.unmodifiableList(this.pois);
	}
	
	/**
	 * Adds a point of interest to these managed goals.
	 * 
	 * @param poi the point of interest to be added
	 * 
	 * @return true if the point of interest was added, false otherwise
	 * 
	 * @throws IllegalArgumentException if the point of interest is null
	 */
	public boolean addPoi(Position poi) {
		if (null == poi) {
			throw new IllegalArgumentException();
		}
		return this.pois.add(poi);
	}
	
	/**
	 * Adds all points of interest to these managed goals.
	 * 
	 * @param pois the points of interest to be added
	 * 
	 * @return true if the points of interest were added, false otherwise
	 * 
	 * @throws IllegalArgumentException if any point of interest is null
	 */
	public boolean addAllPois(Collection<Position> pois) {
		if (pois.contains(null)) {
			throw new IllegalArgumentException();
		}
		return this.pois.addAll(pois);
	}
	
	/**
	 * Clears the points of interest of these managed goals.
	 */
	public void clearPois() {
		this.pois.clear();
	}
	
}
