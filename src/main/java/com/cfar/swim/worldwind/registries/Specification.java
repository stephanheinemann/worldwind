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
package com.cfar.swim.worldwind.registries;

import java.io.Serializable;


import java.util.Objects;

import com.cfar.swim.worldwind.util.Customizable;
import com.cfar.swim.worldwind.util.Identifiable;
import com.cfar.swim.worldwind.util.ResourceBundleLoader;

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
public class Specification<Registree>
implements Identifiable, Customizable<Registree>, Comparable<Specification<Registree>>, Serializable {
	
	/** the default serial identification of this specification */
	private static final long serialVersionUID = 1L;

	/** the identifier of an A320 aircraft specification */
	public static final String AIRCRAFT_A320_ID = "A320";
	
	/** the description of an A320 aircraft specification */
	public static final String AIRCRAFT_A320_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.aircraft.a320.description");
	
	/** the identifier of an H135 aircraft specification */
	public static final String AIRCRAFT_H135_ID = "H135";
	
	/** the description of an H135 aircraft specification */
	public static final String AIRCRAFT_H135_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.aircraft.h135.description");
	
	/** the identifier of an Iris aircraft specification */
	public static final String AIRCRAFT_IRIS_ID = "IRIS";
	
	/** the description of an Iris aircraft specification */
	public static final String AIRCRAFT_IRIS_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.aircraft.iris.description");
	
	/** the identifier of a planning grid environment specification */
	public static final String ENVIRONMENT_PLANNING_GRID_ID = "Planning Grid";
	
	/** the description of a planning grid environment specification */
	public static final String ENVIRONMENT_PLANNING_GRID_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.environment.grid.description");
	
	/** the identifier of a planning roadmap environment specification */
	public static final String ENVIRONMENT_PLANNING_ROADMAP_ID = "Planning Roadmap";
	
	/** the identifier of a planning continuum environment specification */
	public static final String ENVIRONMENT_PLANNING_CONTINUUM_ID = "Planning Continuum";
	
	/** the description of a planning continuum environment specification */
	public static final String ENVIRONMENT_PLANNING_CONTINUUM_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.environment.continuum.description");
	
	/** the identifier of a forward A* planner specification */
	public static final String PLANNER_FAS_ID = "Forward A*";
	
	/** the description of a forward A* planner specification */
	public static final String PLANNER_FAS_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.fas.description");
	
	/** the identifier of a Theta* planner specification */
	public static final String PLANNER_TS_ID = "Theta*";
	
	/** the description of a Theta* planner specification */
	public static final String PLANNER_TS_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.ts.description");
	
	/** the identifier of an ARA* planner specification */
	public static final String PLANNER_ARAS_ID = "ARA*";
	
	/** the description of an ARA* planner specification */
	public static final String PLANNER_ARAS_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.aras.description");
	
	/** the identifier of an AD* planner specification */
	public static final String PLANNER_ADS_ID = "AD*";
	
	/** the description of an AD* planner specification */
	public static final String PLANNER_ADS_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.ads.description");
	
	/** the identifier of an OAD* planner specification */
	public static final String PLANNER_OADS_ID = "OAD*";
	
	/** the description of an OAD* planner specification */
	public static final String PLANNER_OADS_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.oads.description");
	
	/** the identifier of a managed grid planner specification */
	public static final String PLANNER_MGP_ID = "MGP";
	
	/** the identifier of an RRT planner specification */
	public static final String PLANNER_RRT_ID = "RRT";
	
	/** the description of an RRT planner specification */
	public static final String PLANNER_RRT_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.rrt.description");
	
	/** the identifier of an RRT* planner specification */
	public static final String PLANNER_RRTS_ID = "RRT*";
	
	/** the description of an RRT* planner specification */
	public static final String PLANNER_RRTS_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.rrts.description");
	
	/** the identifier of an hRRT planner specification */
	public static final String PLANNER_HRRT_ID = "hRRT";
	
	/** the description of an hRRT planner specification */
	public static final String PLANNER_HRRT_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.hrrt.description");
	
	/** the identifier of an ARRT planner specification */
	public static final String PLANNER_ARRT_ID = "ARRT";
	
	/** the description of an ARRT planner specification */
	public static final String PLANNER_ARRT_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.arrt.description");
	
	/** the identifier of a DRRT planner specification */
	public static final String PLANNER_DRRT_ID = "DRRT";
	
	/** the description of a DRRT planner specification */
	public static final String PLANNER_DRRT_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.drrt.description");
	
	/** the identifier of an ADRRT planner specification */
	public static final String PLANNER_ADRRT_ID = "ADRRT";
	
	/** the description of an ADRRT planner specification */
	public static final String PLANNER_ADRRT_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.adrrt.description");
	
	/** the identifier of an OADRRT planner specification */
	public static final String PLANNER_OADRRT_ID = "OADRRT";
	
	/** the description of an OADRRT planner specification */
	public static final String PLANNER_OADRRT_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.oadrrt.description");
	
	/** the identifier of a Q Learning planner specification */
	public static final String PLANNER_QLP_ID = "QLearningP";
	
	/** the description of a Q learning planner specification */
	public static final String PLANNER_QLP_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.qlp.description");
	
	/** the identifier of a DQN planner specification */
	public static final String PLANNER_DQN_ID = "DQN";
	
	/** the description of a DQN planner specification */
	public static final String PLANNER_DQN_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.planner.dqn.description");
	
	/** the identifier of a managed tree planner specification */
	public static final String PLANNER_MTP_ID = "MTP";
	
	/** the identifier of a dronekit datalink specification */
	public static final String CONNECTION_DATALINK_DRONEKIT_ID = "Dronekit Datalink";
	
	/** the description of a dronekit datalink specification */
	public static final String CONNECTION_DATALINK_DRONEKIT_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.connection.datalink.dronekit.description");
	
	/** the identifier of a mavlink datalink specification */
	public static final String CONNECTION_DATALINK_MAVLINK_ID = "Mavlink Datalink";
	
	/** the description of a mavlink datalink specification */
	public static final String CONNECTION_DATALINK_MAVLINK_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.connection.datalink.mavlink.description");
	
	/** the identifier of a simulated datalink specification */
	public static final String CONNECTION_DATALINK_SIMULATED_ID = "Simulated Datalink";
	
	/** the description of a simulated datalink specification */
	public static final String CONNECTION_DATALINK_SIMULATED_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.connection.datalink.simulated.description");
	
	/** the identifier of a live SWIM connection specification */
	public static final String CONNECTION_SWIM_LIVE_ID = "Live SWIM";
	
	/** the description of a live SWIM connection specification */
	public static final String CONNECTION_SWIM_LIVE_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.connection.swim.live.description");
	
	/** the identifier of a simulated SWIM connection specification */
	public static final String CONNECTION_SWIM_SIMULATED_ID = "Simulated SWIM";
	
	/** the description of a simulated SWIM connection specification */
	public static final String CONNECTION_SWIM_SIMULATED_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.connection.swim.simulated.description");
	
	/** the identifier of a heuristic manager specification */
	public static final String MANAGER_HEURISTIC_ID = "Heuristic Autonomic Manager";
	
	/** the description of a heuristic manager specification */
	public static final String MANAGER_HEURISTIC_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.manager.heuristic.description");
	
	/** the identifier of a SMAC manager specification */
	public static final String MANAGER_SMAC_ID = "SMAC Autonomic Manager";
	
	/** the description of a SMAC manager specification */
	public static final String MANAGER_SMAC_DESCRIPTION = ResourceBundleLoader
			.getDictionaryBundle()
			.getString("specification.manager.smac.description");
	
	/** the identifier of this specification */
	private final String id;
	
	/** the description of this specification */
	private final String description;
	
	/** the properties bean of this specification */
	private Properties<Registree> properties;
	
	/**
	 * Constructs a new specification with an identifier.
	 * 
	 * @param id the identifier of this specification
	 */
	public Specification(String id) {
		this.id = id;
		this.description = "";
	}
	
	/**
	 * Constructs a new specification with an identifier and properties bean.
	 * 
	 * @param id the identifier of this specification
	 * @param properties the properties bean of this specification
	 */
	public Specification(String id, Properties<Registree> properties) {
		this.id = id;
		this.description = "";
		this.properties = properties;
	}
	
	/**
	 * Constructs a new specification with an identifier, description and
	 * properties bean.
	 * 
	 * @param id the identifier of this specification
	 * @param description the description of this specification
	 * @param properties the properties bean of this specification
	 */
	public Specification(
			String id,
			String description,
			Properties<Registree> properties) {
		this.id = id;
		this.description = description;
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
	 * Gets the description of this specification.
	 * 
	 * @return the description of this specification
	 */
	public String getDescription() {
		return this.description;
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
	
	/**
	 * Determines whether or not this specification equals another
	 * specification based on their identifiers and aggregated properties.
	 * 
	 * @param o the other specification
	 * 
	 * @return true, if the identifier and aggregated properties of this
	 *         specification equal the identifier and aggregated properties
	 *         of the other specification, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (o instanceof Specification)) {
			Specification<?> s = (Specification<?>) o;
			equals = (this.id.equals(s.id)
					&& this.description.equals(s.description)
					&& this.properties.equals(s.properties));
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this specification based on its identifier and
	 * aggregated properties.
	 * 
	 * @return the hash code of this specification based on its identifier and
	 *         aggregated properties
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(this.id, this.description, this.properties);
	}

}
