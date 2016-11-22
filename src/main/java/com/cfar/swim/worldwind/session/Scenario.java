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
package com.cfar.swim.worldwind.session;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.util.ArrayList;

import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.thetastar.ThetaStarPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningGrid;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.util.Enableable;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;

/**
 * Realizes a planning scenario.
 * 
 * @author Stephan Heinemann
 *
 */
public class Scenario implements Identifiable, Enableable {
	
	/** the default scenario identifier */
	public static final String DEFAULT_SCENARIO_ID = "Default Scenario";
	
	/** the property change support of this scenario */
	private final PropertyChangeSupport pcs = new PropertyChangeSupport(this);
	
	/** the identifier of this scenario */
	private final String id;
	
	/** indicates whether or not this scenario is enabled */
	private boolean isEnabled = false;
	
	/** the globe of this scenario */
	private Globe globe;
	
	/** the planning sector on the globe of this scenario */
	private Sector sector;
	
	/** the planning environment of this scenario */
	private Environment environment;
	
	/** the aircraft of this scenario */
	private Aircraft aircraft;
	
	/** the planner of this scenario */
	private Planner planner;
	
	/** the waypoints to be visited in the trajectory of this scenario */
	private ArrayList<Waypoint> waypoints;
	
	/** the planned trajectory of this scenario */
	private Trajectory trajectory;
	
	/**
	 * Constructs and initializes a default scenario.
	 */
	public Scenario() {
		this(Scenario.DEFAULT_SCENARIO_ID);	
	}
	
	/**
	 * Constructs and initializes a scenario with a specified scenario identifier.
	 * 
	 * @param id the scenario identifier
	 */
	public Scenario(String id) {
		this.id = id;
		this.init();
	}
	
	/**
	 * Initializes this scenario.
	 */
	public void init() {
		this.globe = new Earth();
		this.sector = new Sector(Angle.ZERO, Angle.POS90, Angle.ZERO, Angle.POS90);
		gov.nasa.worldwind.geom.Box sectorBox = Sector.computeBoundingBox(this.globe, 1.0, this.sector, 0, 500000);
		Box envBox = new Box(sectorBox);
		Cube planningCube = new Cube(envBox.getOrigin(), envBox.getUnitAxes(), envBox.getRLength() / 10);
		this.waypoints = new ArrayList<Waypoint>();
		this.environment = new PlanningGrid(planningCube, 10, 10, 5);
		this.environment.setThreshold(0);
		this.environment.setGlobe(this.globe);
		this.aircraft = new Iris(this.environment.getCenterPosition(), 5000, CombatIdentification.FRIEND);
		this.setPlanner(new ThetaStarPlanner(this.aircraft, this.environment));
		this.setTrajectory(new Trajectory());
	}
	
	/**
	 * Gets the identifier of this scenario.
	 * 
	 * @return the identifier of this scenario
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return this.id;
	}
	
	/**
	 * Enables this scenario.
	 * 
	 * @see Enableable#enable()
	 */
	@Override
	public void enable() {
		this.isEnabled = true;
		this.pcs.firePropertyChange("isEnabled", null, this.isEnabled);
	}
	
	/**
	 * Disables this scenario.
	 * 
	 * @see Enableable#disable()
	 */
	@Override
	public void disable() {
		this.isEnabled = false;
		this.pcs.firePropertyChange("isEnabled", null, this.isEnabled);
	}
	
	/**
	 * Indicates whether or not this scenario is enabled.
	 * 
	 * @return true if this scenario is enabled, false otherwise
	 * 
	 * @see Enableable#isEnabled()
	 */
	@Override
	public boolean isEnabled() {
		return this.isEnabled;
	}
	
	/**
	 * Adds a property change listener to this scenario.
	 * 
	 * @param listener the property change listener to be added
	 */
	public void addPropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener(listener);
	}
	
	/**
	 * Removes a property change listener from this scenario.
	 * 
	 * @param listener the property change listener to be removed
	 */
	public void removePropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.removePropertyChangeListener(listener);
	}
	
	/**
	 * Adds an enabled change listener to this scenario.
	 * 
	 * @param listener the enabled change listener to be added
	 */
	public void addEnabledChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("isEnabled", listener);
	}
	
	/**
	 * Adds a waypoint change listener to this scenario.
	 * 
	 * @param listener the waypoint change listener to be added
	 */
	public void addWaypointsChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("waypoints", listener);
	}
	
	/**
	 * Gets the environment of this scenario.
	 * 
	 * @return the environment of this scenario
	 */
	public Environment getEnvironment() {
		return this.environment;
	}
	
	/**
	 * Sets the environment of this scenario.
	 * 
	 * @param environment the environment to be set
	 */
	public void setEnvironment(Environment environment) {
		this.environment = environment;
	}
	
	/**
	 * Gets the waypoints of this scenario.
	 * 
	 * @return the waypoints of this scenario
	 */
	public Iterable<Waypoint> getWaypoints() {
		// TODO: waypoints are not immutable and external changes could be problematic
		return this.waypoints;
	}
	
	/**
	 * Adds a waypoint to this scenario and sequences its identifier.
	 * 
	 * @param waypoint the waypoint to be added
	 */
	public void addWaypoint(Waypoint waypoint) {
		String designator = Integer.toString(this.waypoints.size());
		waypoint.setDesignator(designator);
		if (waypoint.hasDepiction() && waypoint.getDepiction().hasAnnotation()) {
			waypoint.getDepiction().getAnnotation().setText(designator);
		} 
		this.waypoints.add(waypoint);
		this.pcs.firePropertyChange("waypoints", null, (Iterable<Waypoint>) this.waypoints);
	}
	
	/**
	 * Adds a waypoint at a specified index to this scenario and sequences all
	 * its waypoint identifiers.
	 * 
	 * @param index the waypoint index
	 * @param waypoint the waypoint to be added
	 */
	public void addWaypoint(int index, Waypoint waypoint) {
		this.waypoints.add(index, waypoint);
		this.sequenceWaypoints();
		this.pcs.firePropertyChange("waypoints", null, (Iterable<Waypoint>) this.waypoints);
	}
	
	/**
	 * Updates and replaces a waypoint of this scenario at the present index.
	 * 
	 * @param oldWaypoint the old waypoint to be replaced
	 * @param newWaypoint the new waypoint to replace the old one
	 */
	public void updateWaypoint(Waypoint oldWaypoint, Waypoint newWaypoint) {
		int index = this.waypoints.indexOf(oldWaypoint);
		if (-1 != index) {
			String designator = Integer.toString(index);
			newWaypoint.setDesignator(designator);
			if (newWaypoint.hasDepiction() && newWaypoint.getDepiction().hasAnnotation()) {
				newWaypoint.getDepiction().getAnnotation().setText(designator);
			}
			this.waypoints.remove(index);
			this.waypoints.add(index, newWaypoint);
			this.pcs.firePropertyChange("waypoints", null, (Iterable<Waypoint>) this.waypoints);
		}
	}
	
	/**
	 * Removes a waypoint from this scenario and sequences all its waypoint
	 * identifiers.
	 * 
	 * @param waypoint the waypoint to be removed
	 */
	public void removeWaypoint(Waypoint waypoint) {
		this.waypoints.remove(waypoint);
		this.sequenceWaypoints();
		this.pcs.firePropertyChange("waypoints", null, (Iterable<Waypoint>) this.waypoints);
	}
	
	/**
	 * Removes a waypoint at a specified index from this scenario and sequences
	 * all waypoint identifiers.
	 * 
	 * @param index the index of the waypoint to be removed
	 */
	public void removeWaypoint(int index) {
		this.waypoints.remove(index);
		this.sequenceWaypoints();
		this.pcs.firePropertyChange("waypoints", null, (Iterable<Waypoint>) this.waypoints);
	}
	
	/**
	 * Removes all waypoints of this scenario.
	 */
	public void clearWaypoints() {
		this.waypoints.clear();
		this.pcs.firePropertyChange("waypoints", null, (Iterable<Waypoint>) this.waypoints);
	}
	
	/**
	 * Sequences all waypoints of this scenario.
	 */
	private void sequenceWaypoints() {
		int number = 0;
		for (Waypoint waypoint : this.getWaypoints()) {
			String designator = Integer.toString(number);
			waypoint.setDesignator(designator);
			if (waypoint.hasDepiction() && waypoint.getDepiction().hasAnnotation()) {
				waypoint.getDepiction().getAnnotation().setText(designator);
			}
			number++;
		}
	}
	
	/**
	 * Gets the planner of this scenario.
	 * 
	 * @return the planner of this scenario
	 */
	public Planner getPlanner() {
		return this.planner;
	}

	/**
	 * Sets the planner of this scenario.
	 * 
	 * @param planner the planner of this scenario
	 */
	public void setPlanner(Planner planner) {
		this.planner = planner;
	}

	/**
	 * Gets the planned trajectory of this scenario.
	 * 
	 * @return the planned trajectory of this scenario
	 */
	public Trajectory getTrajectory() {
		return this.trajectory;
	}

	/**
	 * Sets the planned trajectory of this scenario.
	 * 
	 * @param trajectory the planned trajectory of this scenario
	 */
	public void setTrajectory(Trajectory trajectory) {
		this.trajectory = trajectory;
	}

	/**
	 * Indicates whether or not this scenario equals another scenario based on
	 * their identifiers.
	 * 
	 * @param o the other scenario
	 * 
	 * @return true, if the identifier of this scenario equals the
	 *         identifier of the other scenario, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (o instanceof Scenario) {
			equals = this.id.equals(((Scenario) o).id);
		}
	
		return equals;
	}
	
	/**
	 * Gets the hash code of this scenario based on its identifier.
	 * 
	 * @return the hash code of this scenario based on its identifier
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return this.id.hashCode();
	}
	
}
