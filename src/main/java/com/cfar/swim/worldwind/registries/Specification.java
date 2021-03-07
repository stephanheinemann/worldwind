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
package com.cfar.swim.worldwind.registries;

import com.cfar.swim.worldwind.util.Customizable;
import com.cfar.swim.worldwind.util.Identifiable;

/**
 * Realizes a generic customizable specification. Specifications can be passed
 * to factories to create items according to a customized specification such
 * as environments and planners.
 * 
 * @author Stephan Heinemann
 *
 * @param <Registree> the specified (registered) item
 * 
 * @see Factory
 * @see Registry
 */
public class Specification<Registree> implements Identifiable, Customizable<Registree>, Comparable<Specification<Registree>> {
	
	/** the identifier of an Iris aircraft */
	public static final String AIRCRAFT_IRIS_ID = "Iris";
	
	/** the identifier of an A320 aircraft */
	public static final String AIRCRAFT_A320_ID = "A320";
	
	/** the identifier of a planning grid environment */
	public static final String PLANNING_GRID_ID = "Planning Grid";
	
	/** the identifier of a planning roadmap environment */
	public static final String PLANNING_ROADMAP_ID = "Planning Roadmap";
	
	/** the identifier of a planning continuum environment */
	public static final String PLANNING_CONTINUUM_ID = "Planning Continuum";
	
	/** the identifier of a forward A* planner */
	public static final String PLANNER_FAS_ID = "Forward A*";
	
	/** the identifier of a Theta* planner */
	public static final String PLANNER_TS_ID = "Theta*";
	
	/** the identifier of an ARA* planner */
	public static final String PLANNER_ARAS_ID = "ARA*";
	
	/** the identifier of an AD* planner */
	public static final String PLANNER_ADS_ID = "AD*";
	
	/** the identifier of a dronekit datalink */
	public static final String DATALINK_DRONEKIT = "Dronekit Datalink";
	
	/** the identifier of a simulated datalink */
	public static final String DATALINK_SIMULATED = "Simulated Datalink";
	
	/** the identifier of a live SWIM connection */
	public static final String SWIM_LIVE = "Live SWIM";
	
	/** the identifier of a simulated SWIM connection */
	public static final String SWIM_SIMULATED = "Simulated SWIM";
	
	/** the identifier of this specification */
	private final String id;
	
	/** the properties bean of this specification */
	private Properties<Registree> properties;
	
	/**
	 * Constructs a new specification with an identifier.
	 * 
	 * @param id the identifier of this specification
	 */
	public Specification(String id) {
		this.id = id;
	}
	
	/**
	 * Constructs a new Specification with an identifier and properties bean.
	 * 
	 * @param id the identifier of this specification
	 * @param properties the properties bean of this specification
	 */
	public Specification(String id, Properties<Registree> properties) {
		this.id = id;
		this.properties = properties;
	}
	
	/**
	 * Gets the identifier of this specification.
	 * 
	 * @return the identifier of this specification
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return this.id;
	}
	
	/**
	 * Gets the properties bean of this specification.
	 * 
	 * @return the properties bean of this specification
	 * 
	 * @see Customizable#getProperties()
	 */
	@Override
	public Properties<Registree> getProperties() {
		return this.properties;
	}

	/**
	 * Sets the properties bean of this specification.
	 * 
	 * @param properties the properties bean to be set
	 * 
	 * @see Customizable#setProperties(Properties)
	 */
	@Override
	public void setProperties(Properties<Registree> properties) {
		this.properties = properties;
	}

	/**
	 * Compares this specification to another one with respect to their
	 * identifiers.
	 * 
	 * @param specification the other specification
	 * 
	 * @return -1, 0, 1, if the identifier of this specification is
	 *         lexicographically less, equal or greater than the identifier
	 *         of the other specification, respectively
	 * 
	 * @see Comparable#compareTo(Object)
	 * @see String#compareTo(String)
	 */
	@Override
	public int compareTo(Specification<Registree> specification) {
		return this.id.compareTo(specification.getId());
	}

}
