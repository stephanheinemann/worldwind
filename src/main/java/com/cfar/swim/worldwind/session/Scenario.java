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
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;

public class Scenario implements Identifiable {

	// TODO: equals, hashCode (id)
	
	public static final String DEFAULT_SCENARIO_ID = "default";
	private final PropertyChangeSupport pcs = new PropertyChangeSupport(this);
	
	protected String id;
	protected Globe globe;
	protected Sector sector;
	protected Environment environment;
	protected ArrayList<Waypoint> pois;
	protected Aircraft aircraft;
	protected Planner planner;
	protected Trajectory trajectory;
	
	public Scenario() {
		this.id = Scenario.DEFAULT_SCENARIO_ID;
		this.globe = new Earth();
		this.sector = new Sector(Angle.ZERO, Angle.ZERO, Angle.POS90, Angle.POS90);
		gov.nasa.worldwind.geom.Box sectorBox = Sector.computeBoundingBox(this.globe, 1.0, this.sector, 0, 50000);
		Box envBox = new Box(sectorBox);
		Cube planningCube = new Cube(envBox.getOrigin(), envBox.getUnitAxes(), envBox.getRLength() / 10);
		this.pois = new ArrayList<Waypoint>();
		this.environment = new PlanningGrid(planningCube, 10, 10, 5);
		this.environment.setGlobe(this.globe);
		this.aircraft = new Iris(this.environment.getCenterPosition(), 5000, CombatIdentification.FRIEND);
		this.planner = new ThetaStarPlanner(this.aircraft, this.environment);
		this.trajectory = new Trajectory();
	}
	
	public Scenario(String id) {
		this.id = id;
		// TODO: ...
	}
	
	@Override
	public String getId() {
		return this.id;
	}
	
	@Override
	public void setId(String id) {
		this.id = id;
	}

	public void addPropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener(listener);
	}
	
	public void addPointsOfInterestChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("pois", listener);
	}
	
	public void removePropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.removePropertyChangeListener(listener);
	}
	
	public Iterable<Waypoint> getPointsOfInterest() {
		// TODO: waypoints are not immutable and external changes are problematic
		return this.pois;
	}
	
	public void addPointOfInterest(Waypoint waypoint) {
		waypoint.setId(Integer.toString(this.pois.size()));
		this.pois.add(waypoint);
		this.pcs.firePropertyChange("pois", null, (Iterable<Waypoint>) this.pois);
	}
	
	public void addPointOfInterest(int index, Waypoint waypoint) {
		this.pois.add(index, waypoint);
		this.renumberPointsOfInterest();
		this.pcs.firePropertyChange("pois", null, (Iterable<Waypoint>) this.pois);
	}
	
	public void removePointOfInterest(Waypoint waypoint) {
		this.pois.remove(waypoint);
		this.renumberPointsOfInterest();
		this.pcs.firePropertyChange("pois", null, (Iterable<Waypoint>) this.pois);
	}
	
	public void removePointOfInterest(int index) {
		this.pois.remove(index);
		this.renumberPointsOfInterest();
		this.pcs.firePropertyChange("pois", null, (Iterable<Waypoint>) this.pois);
	}
	
	public void clearPointsOfInterest() {
		this.pois.clear();
		this.pcs.firePropertyChange("pois", null, (Iterable<Waypoint>) this.pois);
	}
	
	private void renumberPointsOfInterest() {
		int number = 0;
		for (Waypoint poi : this.getPointsOfInterest()) {
			poi.setId(Integer.toString(number));
			number++;
		}
	}
}
