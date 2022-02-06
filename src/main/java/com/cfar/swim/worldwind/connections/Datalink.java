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
package com.cfar.swim.worldwind.connections;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.DatalinkProperties;
import com.cfar.swim.worldwind.tracks.AircraftTrack;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.util.Logging;

/**
 * Abstracts a datalink connection to connect to and communicate with aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class Datalink implements Connection {
	
	/** the property change support of this datalink */
	private final PropertyChangeSupport pcs = new PropertyChangeSupport(this);
	
	/** the downlink (monitoring) period of this datalink */
	private Duration downlinkPeriod = Duration.ofMillis(1000);
	
	/** the cached mission of this datalink */
	private Path mission = new Path(new ArrayList<>());
	
	/** the track of the source of this datalink */
	private AircraftTrack track = new AircraftTrack();
	
	/** the monitor executor of this datalink */
	private ScheduledExecutorService executor = null;
	
	
	/**
	 * Connects this datalink.
	 * 
	 * @see Connection#connect()
	 */
	@Override
	public abstract void connect();
	
	/**
	 * Disconnect this datalink.
	 * 
	 * @see Connection#connect()
	 */
	@Override
	public abstract void disconnect();
	
	/**
	 * Determines whether or not this datalink is connected.
	 * 
	 * @return true if this datalink is connected, false otherwise
	 * 
	 * @see Connection#isConnected()
	 */
	@Override
	public abstract boolean isConnected();
	
	/**
	 * Emits a heart beat via this datalink.
	 */
	public abstract void emitHeartbeat();
	
	/**
	 * Gets the aircraft status via this datalink.
	 * 
	 * @return the aircraft status obtained via this datalink
	 */
	public abstract String getAircraftStatus();
	
	/**
	 * Gets the aircraft mode via this datalink.
	 * 
	 * @return the aircraft mode obtained via this datalink
	 */
	public abstract String getAircraftMode();
	
	/**
	 * Sets the aircraft mode via this datalink.
	 * 
	 * @param aircraftMode the aircraft mode to be set
	 */
	public abstract void setAircraftMode(String aircraftMode);
	
	/**
	 * Gets the aircraft heading via this datalink.
	 * 
	 * @return the aircraft heading obtained via this datalink
	 */
	public abstract Angle getAircraftHeading();
	
	/**
	 * Gets the aircraft pitch via this datalink.
	 * 
	 * @return the aircraft pitch obtained via this datalink
	 */
	public abstract Angle getAircraftPitch();
	
	/**
	 * Gets the aircraft bank via this datalink.
	 * 
	 * @return the aircraft bank obtained via this datalink
	 */
	public abstract Angle getAircraftBank();
	
	/**
	 * Gets the aircraft yaw via this datalink.
	 * 
	 * @return the aircraft yaw obtained via this datalink
	 */
	public abstract Angle getAircraftYaw();
	
	/**
	 * Gets the aircraft position via this datalink.
	 * 
	 * @return the aircraft position obtained via this datalink
	 */
	public abstract Position getAircraftPosition();
	
	/**
	 * Gets an aircraft track point via this datalink.
	 * 
	 * @return an aircraft track point obtained via this datalink
	 */
	public abstract AircraftTrackPoint getAircraftTrackPoint();
	
	/**
	 * Gets the aircraft track monitored via this datalink.
	 * 
	 * @return the aircraft track monitored via this datalink
	 */
	public AircraftTrack getAircraftTrack() {
		return this.track;
	}
	
	/**
	 * Clears the aircraft track monitored via this datalink.
	 */
	public void clearAircraftTrack() {
		this.track.clear();
		this.pcs.firePropertyChange("track", null, this.track);
	}
	
	/**
	 * Enables the aircraft guidance via this datalink.
	 */
	public abstract void enableAircraftGuidance();
	
	/**
	 * Disables the aircraft guidance via this datalink.
	 */
	public abstract void disableAircraftGuidance();
	
	/**
	 * Determines whether or not the aircraft guidance is enabled for the
	 * aircraft connected via this datalink.
	 * 
	 * @return true if the aircraft guidance is enabled, false otherwise
	 */
	public abstract boolean isAircraftGuidanceEnabled();
	
	/**
	 * Arms the aircraft via this datalink.
	 */
	public abstract void armAircraft();
	
	/**
	 * Disarms the aircraft via this datalink.
	 */
	public abstract void disarmAircraft();
	
	/**
	 * Determines whether or not the aircraft connected via this datalink is
	 * armed.
	 * 
	 * @return true if the aircraft is armed, false otherwise
	 */
	public abstract boolean isAircraftArmed();
	
	/**
	 * Uploads a mission flight path to the aircraft connected via this
	 * datalink.
	 * 
	 * @param mission the mission flight path to be uploaded
	 * 
	 * @throws IllegalArgumentException if the mission is null
	 */
	public void uploadMission(Path mission) {
		if ((null == mission) || (null == mission.getPositions())) {
			throw new IllegalArgumentException("invalid mission");
		}
		this.mission = mission;
	}
	
	/**
	 * Downloads a mission flight path from the aircraft connected via this
	 * datalink.
	 * 
	 * @param cached indicates whether or not the mission cache is used
	 * 
	 * @return the downloaded mission flight path
	 */
	public Path downloadMission(boolean cached) {
		return cached ? this.mission : new Path(new ArrayList<>());
	}
	
	/**
	 * Determines whether or not a mission flight path has been uploaded.
	 * 
	 * @param cached indicates whether or not the mission cache is used
	 * 
	 * @return true if a mission flight path has been uploaded,
	 *         false otherwise
	 */
	public boolean hasMission(boolean cached) {
		return cached ? (!Iterables.isEmpty(this.mission.getPositions()))
				: (!Iterables.isEmpty(this.downloadMission(cached).getPositions()));
	}
	
	/**
	 * Determines whether or not a mission flight path has been uploaded.
	 * 
	 * @param mission the mission flight path to be checked
	 * @param cached indicates whether or not the mission cache is used
	 * 
	 * @return true if the mission flight path has been uploaded,
	 *         false otherwise
	 */
	public boolean hasMission(Path mission, boolean cached) {
		boolean hasMission = false;
		Path downloaded = this.downloadMission(cached);
		
		if ((null != mission) && (null != mission.getPositions())) {
			hasMission = true;
			Iterator<? extends Position> dpi = downloaded.getPositions().iterator();
			Iterator<? extends Position> ppi = mission.getPositions().iterator();
			while (hasMission && dpi.hasNext() && ppi.hasNext()) {
				PrecisionPosition dpp = new PrecisionPosition(dpi.next());
				PrecisionPosition ppp = new PrecisionPosition(ppi.next());
				Logging.logger().info("downloaded = " + dpp + ", prepared = " + ppp);
				if (!dpp.equals(ppp)) {
					hasMission = false;
				}
			}
			if (!hasMission || dpi.hasNext() || ppi.hasNext()) {
				hasMission = false;
			}
		
		}
		
		return hasMission;
	}
	
	/**
	 * Gets the next position of the mission flight path from the aircraft
	 * connected via this datalink.
	 * 
	 * @return the next position of the mission flight path
	 */
	public abstract Position getNextMissionPosition();
	
	/**
	 * Gets the index of the next position of the mission flight path from
	 * the aircraft connected via this datalink.
	 * 
	 * @return the index of the next position of the mission flight path
	 */
	public abstract int getNextMissionPositionIndex();
	
	/**
	 * Initiates a take-off for the aircraft connected via this datalink.
	 */
	public abstract void takeOff();
	
	/**
	 * Initiates a landing for the aircraft connected via this datalink.
	 */
	public abstract void land();
	
	/**
	 * Initiates a return to and landing at the launch position for the
	 * aircraft connected via this datalink.
	 */
	public abstract void returnToLaunch();
	
	/**
	 * Determines whether or not the aircraft connected via this datalink is
	 * airborne.
	 * 
	 * @return true if the aircraft is airborne, false otherwise
	 */
	public abstract boolean isAirborne();
	
	// TODO: take-off specification / setup
	// flight envelope (initial altitude, vertical speed, horizontal speed)
	// isAutonomous (mode)
	// getAttitude (Pitch, Roll, Yaw)
	// getGroundSpeed
	// getAirSpeed (True, Equivalent, Calibrated, Indicated)
	// TODO: have Scenario listen for track changes towards the next position
	// if airborne and update ATO accordingly
	
	/**
	 * Gets the downlink (monitoring) period of this datalink.
	 * 
	 * @return the downlink period of this datalink
	 */
	public Duration getDownlinkPeriod() {
		return this.downlinkPeriod;
	}
	
	/**
	 * Sets the downlink (monitoring) period of this datalink.
	 * 
	 * @param downlinkPeriod the downlink period to be set
	 * 
	 * @throws IllegalArgumentException if the downlink period is invalid
	 */
	public void setDownlinkPeriod(Duration downlinkPeriod) {
		if (!downlinkPeriod.isNegative() && !downlinkPeriod.isZero()) {
			this.downlinkPeriod = downlinkPeriod;
		} else {
			throw new IllegalArgumentException("downlink period is invalid");
		}
	}
	
	/**
	 * Starts the datalink monitor with the current downlink period.
	 */
	public void startMonitoring() {
		if (!this.isMonitoring()) {
			this.clearAircraftTrack();
			this.executor = Executors.newSingleThreadScheduledExecutor();
			this.executor.scheduleAtFixedRate(
					new DatalinkMonitor(),
					0,
					this.getDownlinkPeriod().toMillis(),
					TimeUnit.MILLISECONDS);
		}
	}
	
	/**
	 * Stops the datalink monitor.
	 */
	public void stopMonitoring() {
		if (null != this.executor) {
			this.executor.shutdown();
			this.executor = null;
		}
	}
	
	/**
	 * Determines whether or not this datalink is being monitored.
	 * 
	 * @return true if this datalink is being monitored, false otherwise
	 */
	public boolean isMonitoring() {
		return (null != this.executor);
	}
	
	/**
	 * Realizes a datalink monitor.
	 * 
	 * @author Stephan Heinemann
	 *
	 */
	private class DatalinkMonitor implements Runnable {
		
		/**
		 * Monitors the datalink and fires registered property change listeners.
		 * 
		 * @see Runnable#run()
		 */
		@Override
		public void run() {
			emitHeartbeat();
			
			// clean up old track points
			while (!track.isEmpty() && track.peekFirst().isOld()) {
				track.removeFirst();
			}
			
			// add new track point
			AircraftTrackPoint trackPoint = getAircraftTrackPoint();
			if (null != trackPoint) {
				track.add(trackPoint);
				pcs.firePropertyChange("track", null, track);
			}
			// TODO: extend monitored properties
		}
	}
	
	/**
	 * Adds a property change listener to this datalink.
	 * 
	 * @param listener the property change listener to be added
	 */
	public void addPropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener(listener);
	}
	
	/**
	 * Removes a property change listener from this datalink.
	 * 
	 * @param listener the property change listener to be removed
	 */
	public void removePropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.removePropertyChangeListener(listener);
	}
	
	/**
	 * Adds a track change listener to this datalink.
	 * 
	 * @param listener the position change listener to be added
	 */
	public void addTrackChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("track", listener);
	}
	
	/**
	 * Determines whether or not this datalink matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this datalink matches the specification,
	 *         false otherwise
	 * 
	 * @see FactoryProduct#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification)
				&& specification.getId().equals(this.getId())
				&& (specification.getProperties() instanceof DatalinkProperties)) {
			
			DatalinkProperties dlp = (DatalinkProperties) specification.getProperties();
			matches = (this.getDownlinkPeriod().equals(dlp.getDownlinkPeriod()));
		}
	
		return matches;
	}
	
	/**
	 * Updates this datalink according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this datalink has been updated, false otherwise
	 * 
	 * @see FactoryProduct#update(Specification)
	 */
	@Override
	public boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = false;
		
		if ((null != specification)
				&& specification.getId().equals(this.getId())
				&& (specification.getProperties() instanceof DatalinkProperties)
				&& !this.matches(specification)) {
			
			DatalinkProperties properties = (DatalinkProperties) specification.getProperties();
			this.setDownlinkPeriod(Duration.ofMillis(properties.getDownlinkPeriod()));
			updated = true;
		}
		
		return updated;
	}
	
}
