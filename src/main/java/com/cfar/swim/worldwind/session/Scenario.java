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
package com.cfar.swim.worldwind.session;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.connections.SimulatedDatalink;
import com.cfar.swim.worldwind.connections.SimulatedSwimConnection;
import com.cfar.swim.worldwind.connections.SwimConnection;
import com.cfar.swim.worldwind.environments.DynamicEnvironment;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.MultiResolutionEnvironment;
import com.cfar.swim.worldwind.environments.StructuralChangeListener;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.planners.DynamicObstacleListener;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.cgs.thetastar.ThetaStarPlanner;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.util.Enableable;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;

/**
 * Realizes a planning scenario.
 * 
 * @author Stephan Heinemann
 *
 */
public class Scenario implements Identifiable, Enableable, StructuralChangeListener, ObstacleManager {
	
	/** the default scenario identifier */
	public static final String DEFAULT_SCENARIO_ID = "Default Scenario";
	
	/** the property change support of this scenario */
	private final PropertyChangeSupport pcs = new PropertyChangeSupport(this);
	
	/** the identifier of this scenario */
	private final String id;
	
	/** indicates whether or not this scenario is enabled */
	private boolean isEnabled = false;
	
	/** the time of this scenario */
	private ZonedDateTime time;
	
	/** the time controller of this scenario */
	private TimeController timeController;
	
	/** the cost threshold of this scenario */
	private double threshold;
	
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
	
	/** the datalink of this scenario */
	private Datalink datalink;
	
	/** thie SWIM connection of this scenario */
	private SwimConnection swimConnection;
	
	/** the waypoints to be visited in the trajectory of this scenario */
	private ArrayList<Waypoint> waypoints;
	
	/** the planned trajectory of this scenario */
	private Trajectory trajectory;
	
	/** the obstacles of this scenario */
	private HashSet<Obstacle> obstacles;
	
	/** the pending (uncommitted) obstacles of this scenario */
	private HashSet<Obstacle> pendingObstacles;
	
	/** the timer executor of this scenario */
	private ScheduledExecutorService executor;
	
	
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
	public synchronized void init() {
		// TODO: improve initial scenario
		this.time = ZonedDateTime.now(ZoneId.of("UTC"));
		this.timeController = new TimeController(0);
		this.threshold = 0d;
		this.globe = new Earth();
		this.sector = new Sector(Angle.ZERO, Angle.POS90, Angle.ZERO, Angle.POS90);
		gov.nasa.worldwind.geom.Box sectorBox = Sector.computeBoundingBox(this.globe, 1d, this.sector, 0d, 500000d);
		Box envBox = new Box(sectorBox);
		Cube planningCube = new Cube(envBox.getOrigin(), envBox.getUnitAxes(), envBox.getRLength() / 10);
		this.environment = new PlanningGrid(planningCube, 10, 10, 5);
		this.environment.setThreshold(0d);
		this.environment.setGlobe(this.globe);
		((PlanningGrid) this.environment).addStructuralChangeListener(this);
		this.aircraft = null;
		this.waypoints = new ArrayList<Waypoint>();
		this.setPlanner(new ThetaStarPlanner(this.aircraft, this.environment));
		this.setDatalink(new SimulatedDatalink());
		this.setSwimConnection(new SimulatedSwimConnection());
		this.getSwimConnection().setObstacleManager(this);
		this.setTrajectory(new Trajectory());
		this.obstacles = new HashSet<Obstacle>();
		this.pendingObstacles = new HashSet<Obstacle>();
	}
	
	/**
	 * Gets the identifier of this scenario.
	 * 
	 * @return the identifier of this scenario
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public synchronized String getId() {
		return this.id;
	}
	
	/**
	 * Enables this scenario.
	 * 
	 * @see Enableable#enable()
	 */
	@Override
	public synchronized void enable() {
		this.isEnabled = true;
		this.pcs.firePropertyChange("isEnabled", null, this.isEnabled);
	}
	
	/**
	 * Disables this scenario.
	 * 
	 * @see Enableable#disable()
	 */
	@Override
	public synchronized void disable() {
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
	public synchronized boolean isEnabled() {
		return this.isEnabled;
	}
	
	/**
	 * Adds a property change listener to this scenario.
	 * 
	 * @param listener the property change listener to be added
	 */
	public synchronized void addPropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener(listener);
	}
	
	/**
	 * Removes a property change listener from this scenario.
	 * 
	 * @param listener the property change listener to be removed
	 */
	public synchronized void removePropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.removePropertyChangeListener(listener);
	}
	
	/**
	 * Removes all property change listeners from this scenario.
	 */
	public synchronized void clearPropertyChangeListeners() {
		for (PropertyChangeListener listener : this.pcs.getPropertyChangeListeners()) {
			this.pcs.removePropertyChangeListener(listener);
		}
	}
	
	/**
	 * Adds an enabled change listener to this scenario.
	 * 
	 * @param listener the enabled change listener to be added
	 */
	public synchronized void addEnabledChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("isEnabled", listener);
	}
	
	/**
	 * Adds a time change listener to this scenario.
	 * 
	 * @param listener the time change listener to be added
	 */
	public synchronized void addTimeChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("time", listener);
	}
	
	/**
	 * Adds a cost threshold change listener to this scenario.
	 * 
	 * @param listener the cost threshold change listener to be added
	 */
	public synchronized void addThresholdChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("threshold", listener);
	}
	
	/**
	 * Adds an aircraft change listener to this scenario.
	 * 
	 * @param listener the aircraft change listener to be added
	 */
	public synchronized void addAircraftChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("aircraft", listener);
	} 
	
	/**
	 * Adds an environment change listener to this scenario.
	 * 
	 * @param listener the environment change listener to be added
	 */
	public synchronized void addEnvironmentChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("environment", listener);
	}
	
	/**
	 * Adds a waypoints change listener to this scenario.
	 * 
	 * @param listener the waypoints change listener to be added
	 */
	public synchronized void addWaypointsChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("waypoints", listener);
	}
	
	/**
	 * Adds a trajectory change listener to this scenario.
	 * 
	 * @param listener the trajectory change listener to be added
	 */
	public synchronized void addTrajectoryChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("trajectory", listener);
	}
	
	/**
	 * Adds an obstacles change listener to this scenario.
	 * 
	 * @param listener the obstacles change listener to be added
	 */
	public synchronized void addObstaclesChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("obstacles", listener);
	}
	
	// TODO: be more conservative with firing property change listeners
	// only fire if properties have actually changed and pass along old values
	
	/**
	 * Gets the time of this scenario.
	 * 
	 * @return the time of this scenario
	 */
	public synchronized ZonedDateTime getTime() {
		return this.time;
	}
	
	/**
	 * Sets the time of this scenario.
	 * 
	 * @param time the time to be set
	 * 
	 * @throws IllegalArgumentException if time is null
	 */
	public synchronized void setTime(ZonedDateTime time) {
		if (null != time) {
			this.time = time;
			
			if (this.hasAircraft()) {
				this.aircraft.setTime(time);
				this.moveAircraftOnTrajectory();
			}
			
			this.environment.setTime(time);
			this.obstacles.forEach(o -> o.setTime(time));
			
			// TODO: consider time property change reaction versus firing individual changes
			this.pcs.firePropertyChange("time", null, this.time);
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Starts the time for this scenario with a specified advancement
	 * factor. A factor of zero tracks the current time and a positive
	 * or negative factor advances the time towards the future or past,
	 * respectively.
	 * 
	 * @param factor the advancement factor
	 */
	public synchronized void startTime(int factor) {
		if (!this.isTimed()) {
			this.timeController.reset();
			this.timeController.setFactor(factor);
			this.executor = Executors.newSingleThreadScheduledExecutor();
			this.executor.scheduleAtFixedRate(this.timeController, 0l, 500l, TimeUnit.MILLISECONDS);
		}
	}
	
	/**
	 * Stops the time for this scenario.
	 */
	public synchronized void stopTime() {
		if (null != this.executor) {
			this.executor.shutdown();
			this.executor = null;
		}
	}
	
	/**
	 * Tracks the real time for this scenario.
	 */
	public synchronized void trackTime() {
		this.startTime(0);
	}
	
	/**
	 * Starts the current time for this scenario.
	 */
	public synchronized void playTime() {
		this.startTime(1);
	}
	
	/**
	 * Rewinds the time for this scenario.
	 */
	public synchronized void rewindTime() {
		if (this.isTimePlaying()) {
			int factor = this.timeController.getFactor();
			if (1 < factor) {
				this.timeController.setFactor(factor / 2);
			} else if (1 == factor) {
				this.timeController.setFactor(-1);
			} else {
				this.timeController.setFactor(factor * 2);
			}
		}
	}
	
	/**
	 * Fast forwards the time for this scenario.
	 */
	public synchronized void fastForwardTime() {
		if (this.isTimePlaying()) {
			int factor = this.timeController.getFactor();
			if (-1 > factor) {
				this.timeController.setFactor(factor / 2);
			} else if (-1 == factor) {
				this.timeController.setFactor(1);
			} else {
				this.timeController.setFactor(factor * 2);
			}
		}
	}
	
	/**
	 * Indicates whether or not this scenario is tracking the real time.
	 * 
	 * @return true if this scenario is tracking the real time, false otherwise
	 */
	public synchronized boolean isTimeTracking() {
		return (null != this.executor) && (0 == this.timeController.getFactor());
	}
	
	/**
	 * Indicates whether or not this scenario is playing its current time.
	 * 
	 * @return true if this scenario is playing its current time, false otherwise
	 */
	public synchronized boolean isTimePlaying() {
		return (null != this.executor) && (0 != this.timeController.getFactor());
	}
	
	/**
	 * Indicates whether or not this scenario is being timed.
	 *  
	 * @return true if this scenario is being timed, false otherwise
	 */
	public synchronized boolean isTimed() {
		return (null != this.executor);
	}
	
	/**
	 * Gets the cost threshold of this scenario.
	 * 
	 * @return the cost threshold of this scenario
	 */
	public synchronized double getThreshold() {
		return this.threshold;
	}
	
	/**
	 * Sets the cost threshold of this scenario.
	 * 
	 * @param threshold the cost threshold to be set
	 */
	public synchronized void setThreshold(double threshold) {
		this.threshold = threshold;
		
		if (this.hasAircraft()) {
			this.aircraft.setThreshold(threshold);
		}
		
		this.environment.setThreshold(threshold);
		this.obstacles.forEach(o -> o.setThreshold(threshold));
		
		// TODO: consider threshold property change reaction versus firing individual changes
		this.pcs.firePropertyChange("threshold", null, this.threshold);
	}
	
	/**
	 * Gets the globe of this scenario.
	 * 
	 * @return the globe of this scenario
	 */
	public synchronized Globe getGlobe() {
		return this.globe;
	}
	
	/**
	 * Sets the globe of this scenario.
	 * 
	 * @param globe the globe to be set
	 * 
	 * @throws IllegalArgumentException if globe is null
	 */
	public synchronized void setGlobe(Globe globe) {
		if (null != globe) {
			this.globe = globe;
			this.environment.setGlobe(globe);
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Gets the planning sector of this scenario.
	 * 
	 * @return the planning sector of this scenario
	 */
	public synchronized Sector getSector() {
		return this.sector;
	}
	
	/**
	 * Sets the planning sector of this scenario.
	 * 
	 * @param sector the planning sector to be set
	 * 
	 * @throws IllegalArgumentException if sector is null
	 */
	public synchronized void setSector(Sector sector) {
		if (null != sector) {
			this.sector = sector;
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Gets the aircraft of this scenario.
	 * 
	 * @return the aircraft of this scenario
	 */
	public synchronized Aircraft getAircraft() {
		return this.aircraft;
	}
	
	/**
	 * Sets the aircraft of this scenario.
	 * 
	 * @param aircraft the aircraft to be set
	 */
	public synchronized void setAircraft(Aircraft aircraft) {
		this.aircraft = aircraft;
		
		if (null != this.aircraft) {
			this.aircraft.setTime(this.time);
			this.moveAircraftOnTrajectory();
			this.aircraft.setThreshold(threshold);
		}
		
		this.pcs.firePropertyChange("aircraft", null, this.aircraft);
	}
	
	/**
	 * Removes the aircraft from this scenario.
	 */
	public synchronized void removeAircraft() {
		this.aircraft = null;
		this.pcs.firePropertyChange("aircraft", null, this.aircraft);
	}
	
	/**
	 * Indicates whether or not this scenario has an aircraft.
	 * 
	 * @return true if this scenario has an aircraft, false otherwise
	 */
	public synchronized boolean hasAircraft() {
		return (null != this.aircraft);
	}
	
	/**
	 * Moves the aircraft of this scenario to a specified position.
	 * 
	 * @param position the position the aircraft is moved to
	 */
	public synchronized void moveAircraft(Position position) {
		if (this.hasAircraft()) {
			this.aircraft.moveTo(position);
			this.pcs.firePropertyChange("aircraft", null, this.aircraft);
		}
	}
	
	/**
	 * Moves the aircraft on the trajectory according to the time of this
	 * scenario. If the time of this scenario is before the first or after
	 * the last waypoint of the trajectory, the aircraft is moved to the
	 * first or last waypoint, respectively.
	 */
	public synchronized void moveAircraftOnTrajectory() {
		if (this.hasAircraft() && this.hasTrajectory()) {
			
			// find trajectory leg for the time of this scenario 
			Waypoint previous = null;
			Waypoint next = null;
			Iterator<? extends Waypoint> trajectoryIterator = this.trajectory.getWaypoints().iterator();
			while (trajectoryIterator.hasNext() && (null == next)) {
				next = trajectoryIterator.next();
				if (next.getEto().isBefore(this.time) ) {
					previous = next;
					next = null;
				}
			}
			
			if ((null == previous) && this.hasWaypoints()) {
				// time of this scenario is before the first waypoint
				this.moveAircraft(this.waypoints.get(0));
			} else if ((null == next) && this.hasWaypoints()) {
				// time of this scenario is after the last waypoint
				this.moveAircraft(this.waypoints.get(this.waypoints.size() - 1));
			} else {
				// interpolate between waypoints
				Duration d1 = Duration.between(previous.getEto(), this.time);
				Duration d2 = Duration.between(previous.getEto(), next.getEto());
				double ratio = ((double) d1.getSeconds()) / ((double) d2.getSeconds());
				this.moveAircraft(Position.interpolate(ratio, previous, next));
			}
		}
	}
	
	// TODO: consider elimination of all explicit notifications if possible
	
	/**
	 * Notifies this scenario about a changed aircraft.
	 */
	public synchronized void notifyAircraftChange() {
		this.pcs.firePropertyChange("aircraft", null, this.aircraft);
	}
	
	/**
	 * Gets the environment of this scenario.
	 * 
	 * @return the environment of this scenario
	 */
	public synchronized Environment getEnvironment() {
		return this.environment;
	}
	
	/**
	 * Sets the environment of this scenario.
	 * 
	 * @param environment the environment to be set
	 * 
	 * @throws IllegalArgumentException if environment is null
	 */
	public synchronized void setEnvironment(Environment environment) {
		if (null != environment) {
			this.environment = environment;
			this.environment.setGlobe(this.globe);
			this.environment.setTime(this.time);
			this.environment.setThreshold(this.threshold);
			
			// embed obstacles into dynamic environments
			if (this.environment instanceof DynamicEnvironment) {
				((DynamicEnvironment) this.environment).unembedAll();
				for (Obstacle obstacle : this.obstacles) {
					if (obstacle.isEnabled()) {
						((DynamicEnvironment) this.environment)
						.embed(obstacle);
					}
				}
			}
			
			// be notified about structural changes of multi-resolution environments
			if (this.environment instanceof MultiResolutionEnvironment) {
				((MultiResolutionEnvironment) this.environment)
				.addStructuralChangeListener(this);
			}
		
			this.pcs.firePropertyChange("environment", null, this.environment);
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Notifies this scenario about a changed environment.
	 */
	public synchronized void notifyEnvironmentChange() {
		this.pcs.firePropertyChange("environment", null, this.environment);
	}
	
	/**
	 * Notifies this scenario about a structural environment change.
	 * 
	 * @see StructuralChangeListener#notifyStructuralChange()
	 */
	@Override
	public synchronized void notifyStructuralChange() {
		this.notifyEnvironmentChange();
	}
	
	/**
	 * Gets the waypoints of this scenario.
	 * 
	 * @return the waypoints of this scenario as immutable list
	 */
	@SuppressWarnings("unchecked")
	public synchronized List<Waypoint> getWaypoints() {
		// needs to be immutable: an unmodifiable clone
		return Collections.unmodifiableList((List<Waypoint>) this.waypoints.clone());
	}
	
	/**
	 * Adds a waypoint to this scenario and sequences its identifier.
	 * 
	 * @param waypoint the waypoint to be added
	 * 
	 * @throws IllegalArgumentException if waypoint is null
	 */
	public synchronized void addWaypoint(Waypoint waypoint) {
		if (null != waypoint) {
			String designator = Integer.toString(this.waypoints.size());
			waypoint.setDesignator(designator);
			if (waypoint.hasDepiction() && waypoint.getDepiction().hasAnnotation()) {
				waypoint.getDepiction().getAnnotation().setText(designator);
			} 
			this.waypoints.add(waypoint);
			this.pcs.firePropertyChange("waypoints", null, this.getWaypoints());
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Adds a waypoint at a specified index to this scenario and sequences all
	 * its waypoint identifiers.
	 * 
	 * @param index the waypoint index
	 * @param waypoint the waypoint to be added
	 * 
	 * @throws IllegalArgumentException if waypoint is null
	 */
	public synchronized void addWaypoint(int index, Waypoint waypoint) {
		if (null != waypoint) {
			this.waypoints.add(index, waypoint);
			this.sequenceWaypoints();
			this.pcs.firePropertyChange("waypoints", null, this.getWaypoints());
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Updates and replaces a waypoint of this scenario at the present index.
	 * 
	 * @param oldWaypoint the old waypoint to be replaced
	 * @param newWaypoint the new waypoint to replace the old one
	 * 
	 * @throws IllegalArgumentException if any waypoint is null
	 */
	public synchronized void updateWaypoint(Waypoint oldWaypoint, Waypoint newWaypoint) {
		if ((null != oldWaypoint) && (null != newWaypoint)) {
			int index = this.waypoints.indexOf(oldWaypoint);
			if (-1 != index) {
				String designator = Integer.toString(index);
				newWaypoint.setDesignator(designator);
				if (newWaypoint.hasDepiction() && newWaypoint.getDepiction().hasAnnotation()) {
					newWaypoint.getDepiction().getAnnotation().setText(designator);
				}
				this.waypoints.remove(index);
				this.waypoints.add(index, newWaypoint);
				this.pcs.firePropertyChange("waypoints", null, this.getWaypoints());
			}
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Removes a waypoint from this scenario and sequences all its waypoint
	 * identifiers.
	 * 
	 * @param waypoint the waypoint to be removed
	 * 
	 * @throws IllegalArgumentException if waypoint is null
	 */
	public synchronized void removeWaypoint(Waypoint waypoint) {
		if (null != waypoint) {
			this.waypoints.remove(waypoint);
			this.sequenceWaypoints();
			this.pcs.firePropertyChange("waypoints", null, this.getWaypoints());
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Removes a waypoint at a specified index from this scenario and sequences
	 * all waypoint identifiers.
	 * 
	 * @param index the index of the waypoint to be removed
	 */
	public synchronized void removeWaypoint(int index) {
		this.waypoints.remove(index);
		this.sequenceWaypoints();
		this.pcs.firePropertyChange("waypoints", null, this.getWaypoints());
	}
	
	/**
	 * Removes all waypoints of this scenario.
	 */
	public synchronized void clearWaypoints() {
		this.waypoints.clear();
		this.pcs.firePropertyChange("waypoints", null, this.getWaypoints());
	}
	
	/**
	 * Indicates whether or not this scenario has waypoints.
	 * 
	 * @return true if this scenario has waypoints, false otherwise
	 */
	public synchronized boolean hasWaypoints() {
		return !this.waypoints.isEmpty();
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
	public synchronized Planner getPlanner() {
		return this.planner;
	}

	/**
	 * Sets the planner of this scenario.
	 * 
	 * @param planner the planner to be set
	 * 
	 * @throws IllegalArgumentException if planner is null
	 */
	public synchronized void setPlanner(Planner planner) {
		if (null != planner) {
			this.planner = planner;
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Gets the datalink of this scenario.
	 * 
	 * @return the datalink of this scenario
	 */
	public synchronized Datalink getDatalink() {
		return this.datalink;
	}

	/**
	 * Sets the datalink of this scenario.
	 * 
	 * @param datalink the datalink to be set
	 * 
	 * @throws IllegalArgumentException if datalink is null
	 */
	public synchronized void setDatalink(Datalink datalink) {
		if (null != datalink) {
			this.datalink = datalink;
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Gets the SWIM connection of this scenario.
	 * 
	 * @return the SWIM connection of this scenario
	 */
	public synchronized SwimConnection getSwimConnection() {
		return this.swimConnection;
	}

	/**
	 * Sets the SWIM connection of this scenario.
	 * 
	 * @param swimConnection the SWIM connection to be set
	 * 
	 * @throws IllegalArgumentException if swimConnection is null
	 */
	public synchronized void setSwimConnection(SwimConnection swimConnection) {
		if (null != swimConnection) {
			this.swimConnection = swimConnection;
		} else {
			throw new IllegalArgumentException();
		}
	}

	/**
	 * Gets the planned trajectory of this scenario.
	 * 
	 * @return the planned trajectory of this scenario
	 */
	public synchronized Trajectory getTrajectory() {
		// the waypoints of a trajectory are immutable (createTrajectory)
		return this.trajectory;
	}

	/**
	 * Sets the planned trajectory of this scenario.
	 * 
	 * @param trajectory the planned trajectory of this scenario
	 * 
	 * @throws IllegalArgumentException if trajectory is null
	 */
	public synchronized void setTrajectory(Trajectory trajectory) {
		if (null != trajectory) {
			this.trajectory = trajectory;
			this.sequenceTrajectory();
			this.moveAircraftOnTrajectory();
			this.pcs.firePropertyChange("trajectory", null, this.getTrajectory());
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Clears the planned trajectory of this scenario.
	 */
	public synchronized void clearTrajectory() {
		this.trajectory = new Trajectory();
		this.pcs.firePropertyChange("trajectory", null, this.getTrajectory());
	}
	
	/**
	 * Indicates whether or not this scenario has a computed trajectory.
	 * 
	 * @return true if this scenario has a computed trajectory, false otherwise
	 */
	public synchronized boolean hasTrajectory() {
		return (null != this.trajectory.getWaypoints());
	}
	
	/**
	 * Sequences the trajectory of this scenario.
	 */
	private void sequenceTrajectory() {
		if (this.hasTrajectory()) {
			Iterator<Waypoint> waypointsIterator = this.waypoints.iterator();
			Iterator<? extends Waypoint> trajectoryIterator = this.trajectory.getWaypoints().iterator();
			int number = 0;
			
			if (waypointsIterator.hasNext()) {
				Waypoint current = waypointsIterator.next();
				
				while (waypointsIterator.hasNext()) {
					Waypoint next = waypointsIterator.next();
					boolean isNext = false;
					
					while (trajectoryIterator.hasNext() && !isNext) {
						Waypoint waypoint = trajectoryIterator.next();
						if (waypoint.equals(next)) {
							number = 0;
							current = next;
							isNext = true;
						}
						waypoint.setDesignator(current.getDesignator() + "." + Integer.toString(number));
						number++;
					}
				}
			}
		}
	}
	
	/**
	 * Gets a trajectory leg of the computed trajectory (sub-trajectory)
	 * of this scenario.
	 * 
	 * @param from the first waypoint of the trajectory leg
	 * @param to the last waypoint of the trajectory leg
	 * 
	 * @return the trajectory leg between the first and the last waypoint
	 * 
	 *  @throws IllegalArgumentException if any waypoint is null
	 */
	public synchronized List<Waypoint> getTrajectoryLeg(Waypoint from, Waypoint to) {
		List<Waypoint> leg = new ArrayList<Waypoint>();
		
		if ((null != from) && (null != to)) {
			if (this.hasTrajectory()) {
				Iterator<? extends Waypoint> trajectoryIterator = this.trajectory.getWaypoints().iterator();
				boolean passedFrom = false;
				boolean passedTo = false;
				while (trajectoryIterator.hasNext() && !passedTo) {
					Waypoint waypoint = trajectoryIterator.next();
					if (waypoint.equals(to)) {
						passedTo = true;
						leg.add(waypoint);
					} else if (waypoint.equals(from)) {
						passedFrom = true;
						leg.add(waypoint);
					} else if (passedFrom) {
						leg.add(waypoint);
					}
				}
			}
		} else {
			throw new IllegalArgumentException();
		}
		
		return leg;
	}
	
	/**
	 * Gets the obstacles of this scenario.
	 * 
	 * @return the obstacles of this scenario as immutable set
	 */
	@SuppressWarnings("unchecked")
	public synchronized Set<Obstacle> getObstacles() {
		// needs to be immutable: an unmodifiable clone
		return Collections.unmodifiableSet((Set<Obstacle>) this.obstacles.clone());
	}
	
	/**
	 * Adds an obstacle to this scenario.
	 * 
	 * @param obstacle the obstacle to be added
	 * 
	 * @throws IllegalArgumentException if obstacle is null
	 */
	public synchronized void addObstacle(Obstacle obstacle) {
		if (null != obstacle) {
			if (this.obstacles.add(obstacle)) {
				obstacle.setTime(this.time);
				obstacle.setThreshold(this.threshold);
				this.pcs.firePropertyChange("obstacles", null, this.getObstacles());
				
				if (this.environment instanceof DynamicEnvironment) {
					if (obstacle.isEnabled()
							&& ((DynamicEnvironment) this.environment).embed(obstacle)) {
						this.pcs.firePropertyChange("environment", null, this.environment);
					}
				}
			}
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Removes an obstacle from this scenario.
	 * 
	 * @param obstacle the obstacle to be added
	 * 
	 * @throws IllegalArgumentException if obstacle is null
	 */
	public synchronized void removeObstacle(Obstacle obstacle) {
		if (null != obstacle) {
			if (this.obstacles.remove(obstacle)) {
				this.pcs.firePropertyChange("obstacles", null, this.getObstacles());
				
				if (this.environment instanceof DynamicEnvironment) {
					if (((DynamicEnvironment) this.environment).unembed(obstacle)) {
						this.pcs.firePropertyChange("environment", null, this.environment);
					}
				}
			}
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Removes obstacles with a specified cost interval identifier.
	 * 
	 * @param costIntervalId the cost interval identifier
	 */
	public synchronized void removeObstacles(String costIntervalId) {
		boolean removed = false;
		boolean unembedded = false;
		
		Iterator<Obstacle> obstaclesIterator = this.obstacles.iterator();
		while (obstaclesIterator.hasNext()) {
			Obstacle obstacle = obstaclesIterator.next();
			if (obstacle.getCostInterval().getId().equals(costIntervalId)) {
				obstaclesIterator.remove();
				removed = true;
				
				if (this.environment instanceof DynamicEnvironment) {
					if (((DynamicEnvironment) this.environment).unembed(obstacle)) {
						unembedded = true;
					}
				}
			}
		}
		
		if (removed) {
			this.pcs.firePropertyChange("obstacles", null, this.getObstacles());
		}
		
		if (unembedded) {
			this.pcs.firePropertyChange("environment", null, this.environment);
		}
	}
	
	/**
	 * Removes all obstacles from this scenario.
	 */
	public synchronized void clearObstacles() {
		this.obstacles.clear();
		this.pcs.firePropertyChange("obstacles", null, this.getObstacles());
		
		if (this.environment instanceof DynamicEnvironment) {
			((DynamicEnvironment) this.environment).unembedAll();
			this.pcs.firePropertyChange("environment", null, this.environment);
		}
	}
	
	/**
	 * Embeds an obstacle into the environment of this scenario.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @throws IllegalArgumentException if obstacle is null
	 */
	public synchronized void embedObstacle(Obstacle obstacle) {
		if (null != obstacle) {
			if (this.obstacles.add(obstacle)) {
				obstacle.setTime(this.time);
				obstacle.setThreshold(this.threshold);
				this.pcs.firePropertyChange("obstacles", null, this.getObstacles());
			}
			
			if (this.environment instanceof DynamicEnvironment) {
				if (((DynamicEnvironment) this.environment).embed(obstacle)) {
					this.pcs.firePropertyChange("environment", null, this.environment);
				}
			}
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Unembeds an obstacle from the environment of this scenario.
	 * 
	 * @param obstacle the obstacle to be unembedded
	 * 
	 * @throws IllegalArgumentException if obstacle is null
	 */
	public synchronized void unembedObstacle(Obstacle obstacle) {
		if (null != obstacle) {
			if (this.environment instanceof DynamicEnvironment) {
				if (((DynamicEnvironment) this.environment).unembed(obstacle)) {
					this.pcs.firePropertyChange("environment", null, this.environment);
				}
			}
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	/**
	 * Enables obstacles with a specified cost interval identifier.
	 * 
	 * @param costIntervalId the cost interval identifier
	 */
	public synchronized void enableObstacles(String costIntervalId) {
		boolean enabled = false;
		boolean embedded = false;
		
		for (Obstacle obstacle : this.obstacles) {
			if (obstacle.getCostInterval().getId().equals(costIntervalId)) {
				obstacle.enable();
				enabled = true;
				
				if (this.environment instanceof DynamicEnvironment) {
					if (((DynamicEnvironment) this.environment).embed(obstacle)) {
						embedded = true;
					}
				}
			}
		}
		
		if (enabled) {
			this.pcs.firePropertyChange("obstacles", null, this.getObstacles());
		}
		
		if (embedded) {
			this.pcs.firePropertyChange("environment", null, this.environment);
		}
	}
	
	/**
	 * Disables obstacles with a specified cost interval identifier.
	 * 
	 * @param costIntervalId the cost interval identifier
	 */
	public synchronized void disableObstacles(String costIntervalId) {
		boolean disabled = false;
		boolean unembedded = false;
		
		for (Obstacle obstacle : this.obstacles) {
			if (obstacle.getCostInterval().getId().equals(costIntervalId)) {
				obstacle.disable();
				disabled = true;
				
				if (this.environment instanceof DynamicEnvironment) {
					if (((DynamicEnvironment) this.environment).unembed(obstacle)) {
						unembedded = true;
					}
				}
			}
		}
		
		if (disabled) {
			this.pcs.firePropertyChange("obstacles", null, this.getObstacles());
		}
		
		if (unembedded) {
			this.pcs.firePropertyChange("environment", null, this.environment);
		}
	}
	
	/**
	 * Notifies this scenario about a changed obstacles.
	 */
	public synchronized void notifyObstaclesChange() {
		this.pcs.firePropertyChange("obstacles", null, this.getObstacles());
	}
	
	/**
	 * Submits an obstacle change (addition or removal) to this scenario.
	 * 
	 * @param obstacles the obstacles to be changed
	 * 
	 * @see ObstacleManager#submitObstacleChange(Set)
	 */
	@Override
	public synchronized void submitObstacleChange(Set<Obstacle> obstacles) {
		this.pendingObstacles.addAll(obstacles);
		this.pendingObstacles.removeAll(this.obstacles);
		// only notify dynamic obstacle listener about new obstacles
		if ((!this.pendingObstacles.isEmpty())
				&& this.getPlanner() instanceof DynamicObstacleListener) {
			((DynamicObstacleListener) this.getPlanner()).notifyPendingObstacleChange();
		}
	}
	
	/**
	 * Commits an obstacle change (addition or removal) to this scenario.
	 * 
	 * @return the obstacles that were changed
	 * 
	 * @see ObstacleManager#commitObstacleChange()
	 */
	@Override
	public synchronized Set<Obstacle> commitObstacleChange() {
		Set<Obstacle> committedObstacles = new HashSet<Obstacle>();
		for (Obstacle obstacle : this.pendingObstacles) {
			if (!this.getObstacles().contains(obstacle)) {
				this.addObstacle(obstacle);
				committedObstacles.add(obstacle);
			}
		}
		this.pendingObstacles.clear();
		return committedObstacles;
	}
	
	/**
	 * Retracts an obstacle change (addition or removal) from this scenario.
	 * 
	 * @see ObstacleManager#retractObstacleChange()
	 */
	@Override
	public synchronized void retractObstacleChange() {
		this.pendingObstacles.clear();
	}
	
	/**
	 * Determines whether or not this scenario has an obstacle change.
	 * 
	 * @return true if this scenario has an obstacle change, false otherwise
	 * 
	 * @see ObstacleManager#hasObstacleChange()
	 */
	@Override
	public synchronized boolean hasObstacleChange() {
		return !this.pendingObstacles.isEmpty();
	}
	
	/**
	 * Determines whether or not this scenario equals another scenario based on
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
	public synchronized final boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (o instanceof Scenario)) {
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
	public synchronized final int hashCode() {
		return this.id.hashCode();
	}
	
	/**
	 * Realizes a time controller for this scenario.
	 * 
	 * @author Stephan Heinemann
	 *
	 */
	private class TimeController implements Runnable {
		
		/** the factor by which this time controller advances the time */
		private int factor = 0;
		
		/** the last invocation time of this time controller */
		private ZonedDateTime last = null;
		
		/**
		 * Constructs a new time controller that advances the scenario
		 * time by a specified factor. A factor of zero tracks the
		 * current time.
		 * 
		 * @param factor the advancement factor of this time controller 
		 */
		public TimeController(int factor) {
			this.factor = factor;
		}
		
		/**
		 * Resets this time controller.
		 */
		public void reset() {
			this.last = null;
		}
		
		/**
		 * Gets the advancement factor of this time controller.
		 * 
		 * @return the advancement factor of this time controller
		 */
		public int getFactor() {
			return this.factor;
		}
		
		/**
		 * Sets the advancement factor of this time controller.
		 * 
		 * @param factor the advancement factor to be set
		 */
		public void setFactor(int factor) {
			this.factor = factor;
		}

		/**
		 * Advances the time as specified by the advancement factor.
		 * 
		 * @see Runnable#run()
		 */
		@Override
		public void run() {
			ZonedDateTime now = ZonedDateTime.now(ZoneId.of("UTC"));
			if (0 == factor) {
				setTime(now);
			} else if (null == this.last) {
				this.last = now;
			} else {
				Duration duration = Duration.between(this.last, now);
				Duration advance = duration.multipliedBy(factor);
				setTime(getTime().plus(advance));
				this.last = now;
			}
		}
	}
	
}
