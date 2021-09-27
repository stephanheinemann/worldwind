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

import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.Iterator;

import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.SimulatedDatalinkProperties;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a very simplified simulated datalink to an Iris quadcopter.
 * 
 * @author Stephan Heinemann
 *
 */
public class SimulatedDatalink extends Datalink {
	
	/** the unknown status of an aircraft connected via a simulated datalink */
	public static final String STATUS_UNKNOWN = "UNKNOWN";
	
	/** the active status of an aircraft connected via a simulated datalink */
	public static final String STATUS_ACTIVE = "ACTIVE";
	
	/** the unknown mode of an aircraft connected via a simulated datalink */
	public static final String MODE_UNKNOWN = "UNKNOWN";
	
	/** the auto mode of an aircraft connected via a simulated datalink */
	public static final String MODE_AUTO = "AUTO";
	
	/** the land mode of an aircraft connected via a simulated datalink */
	public static final String MODE_LAND = "LAND";
	
	/** the return to land mode of an aircraft connected via a simulated datalink */
	public static final String MODE_RTL = "RTL";
	
	/** the stabilize mode of an aircraft connected via a simulated datalink */
	public static final String MODE_STABILIZE = "STABILIZE";
	
	// TODO: available status and mode constants (consider enum)
	
	/** the gobe of this simulated datalink */
	private Globe globe = new Earth();
	
	/** the home position of this simulated datalink source */
	private Position homePosition = new Position(Angle.ZERO, Angle.ZERO, 0d);
	
	/** the last position of this simulated datalink source */
	private Position lastPosition = this.homePosition;
	
	/** the next position of this simulated datalink source */
	private Position nextPosition = this.homePosition;
	
	/** the Iris quadcopter being the source of this simulated datalink */
	private Iris iris = new Iris(homePosition, 10d, CombatIdentification.FRIEND);
	
	/** the planned flight path of this simulated datalink source */
	private Path flightPath = null;
	
	/** the planned flight path iterator of this simulated datalink source */
	private Iterator<? extends Position> positionIterator = null;
	
	/** the index of the next position in the planned flight path */
	private int positionIndex = 0;
	
	/** the last reporting time of this simulated datalink */
	private ZonedDateTime reportingTime = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the estimated time over the next position in the planned flight path */
	private ZonedDateTime eto = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the estimated time enroute on the current leg in the planned flight path */
	private Duration ete = Duration.ZERO;
	
	/** indicates whether or not the source of this simulated datalink is airborne */
	private boolean isAirborne = false;
	
	/** indicates whether or not this simulated datalink is connected */
	private boolean isConnected = false;
	
	/** indicates whether or not the source of this simulated datalink is armed */
	private boolean isArmed = false;
	
	/** indicates whether or not the source of this simulated datalink is safe */
	private boolean isAircraftSafetyEnabled = true;
	
	/** the aircraft status of this simulated datalink source */
	private String aircraftStatus = SimulatedDatalink.STATUS_UNKNOWN;
	
	/** the aircraft mode of this simulated datalink source */
	private String aircraftMode = SimulatedDatalink.MODE_UNKNOWN;
	
	/** the maximum aircraft track error of this simulated datalink source */
	private AircraftTrackError maxTrackError = AircraftTrackError.ZERO;
	
	/** the error probability of this simulated datalink source */
	private float errorProbability = 0f;
	
	/**
	 * Constructs a new simulated datatlink.
	 */
	public SimulatedDatalink() {
		this.getAircraftTrack().setName("Simulated Iris");
	}
	
	/**
	 * Gets the identifier of this simulated datalink.
	 * 
	 * @return the identifier of this simulated datalink
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.CONNECTION_DATALINK_SIMULATED_ID;
	}
	
	/**
	 * Connects this simulated datalink.
	 * 
	 * @see Datalink#connect()
	 */
	@Override
	public void connect() {
		this.isConnected = true;
	}
	
	/**
	 * Disconnects this simulated datalink.
	 * 
	 * @see Datalink#disconnect()
	 */
	@Override
	public void disconnect() {
		this.isConnected = false;
	}
	
	/**
	 * Determines whether or not this simulated datalink is connected.
	 * 
	 * @return true if this simulated datalink is connected, false otherwise
	 * 
	 * @see Datalink#isConnected()
	 */
	@Override
	public boolean isConnected() {
		return this.isConnected;
	}
	
	// TODO: check isConnected for entire logic!
	
	/**
	 * Gets the aircraft status via this simulated datalink.
	 * 
	 * @return the aircraft status obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftStatus()
	 */
	@Override
	public synchronized String getAircraftStatus() {
		return this.aircraftStatus;
	}
	
	/**
	 * Gets the aircraft mode via this simulated datalink.
	 * 
	 * @return the aircraft mode obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftMode()
	 */
	@Override
	public synchronized String getAircraftMode() {
		return this.aircraftMode;
	}
	
	/**
	 * Sets the aircraft mode via this simulated datalink.
	 * 
	 * @param aircraftMode the aircraft mode to be set
	 * 
	 * @see Datalink#getAircraftMode()
	 */
	@Override
	public synchronized void setAircraftMode(String aircraftMode) {
		this.aircraftMode = aircraftMode;
	}
	
	/**
	 * Gets the aircraft heading via this simulated datalink.
	 * 
	 * @return the aircraft heading obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftHeading()
	 */
	@Override
	public synchronized Angle getAircraftHeading() {
		return Position.linearAzimuth(this.iris.getReferencePosition(), this.nextPosition);
	}
	
	/**
	 * Gets the aircraft pitch via this simulated datalink.
	 * 
	 * @return the aircraft pitch obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftPitch()
	 */
	@Override
	public Angle getAircraftPitch() {
		return Angle.ZERO;
	}
	
	/**
	 * Gets the aircraft bank via this simulated datalink.
	 * 
	 * @return the aircraft bank obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftBank()
	 */
	@Override
	public Angle getAircraftBank() {
		return Angle.ZERO;
	}
	
	/**
	 * Gets the aircraft yaw via this simulated datalink.
	 * 
	 * @return the aircraft yaw obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftYaw()
	 */
	@Override
	public Angle getAircraftYaw() {
		return Angle.ZERO;
	}
	
	/**
	 * Gets the aircraft position via this simulated datalink.
	 * 
	 * @return the aircraft position obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftPosition()
	 */
	@Override
	public synchronized Position getAircraftPosition() {
		Position currentPosition = this.iris.getReferencePosition();
		this.reportingTime = ZonedDateTime.now(ZoneId.of("UTC"));
		
		if ((null != this.flightPath) && this.isAirborne) {
			// progress mission
			while (!this.reportingTime.isBefore(this.eto) || (-1 == this.positionIndex)) {
				this.lastPosition = this.nextPosition;
				if (this.positionIterator.hasNext()) {
					this.positionIndex++;
					this.nextPosition = this.positionIterator.next();
					this.ete = this.iris.getCapabilities().getEstimatedDuration(
							lastPosition, nextPosition, this.globe);
					
					if (0 == this.positionIndex) {
						// updated mission
						this.eto = this.reportingTime.plus(ete);
					} else {
						// existing mission
						this.eto = this.eto.plus(ete);
					}
				} else {
					break;
				}
			}
			
			// determine current position
			Duration ttg = Duration.between(this.reportingTime, this.eto);
			if (!ttg.isNegative() && !this.ete.isZero()) {
				double ratio = ((double) ttg.toSeconds()) / ((double) this.ete.toSeconds());
				currentPosition = Position.interpolate(ratio, this.nextPosition, this.lastPosition);
			}
			
			// initiate return to land auto-land
			if (this.lastPosition.equals(this.nextPosition)
					&& this.lastPosition.equals(this.homePosition)
					&& this.getAircraftMode().equals(SimulatedDatalink.MODE_RTL)) {
				this.isAirborne = false;
				this.setAircraftMode(SimulatedDatalink.MODE_UNKNOWN);
			}
			
			// TODO: introduce probabilistic errors
			/*
			double xte = this.getMaxTrackError().getCrossTrackError() *
					Math.random() * this.getErrorProbablilty();
			currentPosition = new Position(
					currentPosition.latitude,
					currentPosition.longitude,
					currentPosition.elevation + xte);
			*/
			
		} else if (!this.isAirborne) {
			double elevation = this.globe.getElevationModel()
					.getElevation(currentPosition.latitude, currentPosition.longitude);
			currentPosition = new Position(
					currentPosition.latitude, currentPosition.longitude, elevation);
		}
		
		this.iris.moveTo(currentPosition);
		
		// TODO: observed invalid position (NaN or Infinite components)
		return currentPosition;
	}
	
	/**
	 * Gets an aircraft track point via this simulated datalink.
	 * 
	 * @return an aircraft track point obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftTrackPoint()
	 */
	@Override
	public synchronized AircraftTrackPoint getAircraftTrackPoint() {
		AircraftTrackPoint trackPoint = new AircraftTrackPoint(this.getAircraftPosition());
		trackPoint.setPitch(this.getAircraftPitch());
		trackPoint.setBank(this.getAircraftBank());
		trackPoint.setHeading(this.getAircraftHeading());
		trackPoint.setAto(this.reportingTime);
		return trackPoint;
	}
	
	/**
	 * Enables the aircraft safety via this simulated datalink.
	 * 
	 * @see Datalink#enableAircraftSafety()
	 */
	@Override
	public synchronized void enableAircraftSafety() {
		this.isAircraftSafetyEnabled = true;
	}
	
	/**
	 * Disables the aircraft safety via this simulated datalink.
	 * 
	 * @see Datalink#disableAircraftSafety()
	 */
	@Override
	public synchronized void disableAircraftSafety() {
		this.isAircraftSafetyEnabled = false;
	}
	
	/**
	 * Determines whether or not the aircraft safety is enabled for the aircraft
	 * connected via this simulated datalink.
	 * 
	 * @return true if the aircraft safety is enabled, false otherwise
	 * 
	 * @see Datalink#isAircraftSafetyEnabled()
	 */
	@Override
	public synchronized boolean isAircraftSafetyEnabled() {
		return this.isAircraftSafetyEnabled;
	}
	
	/**
	 * Arms the aircraft via this simulated datalink.
	 * 
	 * @see Datalink#armAircraft()
	 */
	@Override
	public synchronized void armAircraft() {
		this.isArmed = true;
	}
	
	/**
	 * Disarms the aircraft via this simulated datalink.
	 * 
	 * @see Datalink#disarmAircraft()
	 */
	@Override
	public synchronized void disarmAircraft() {
		this.isArmed = false;
	}
	
	/**
	 * Determines whether or not the aircraft connected via this simulated
	 * datalink is armed.
	 * 
	 * @return true if the aircraft is armed, false otherwise
	 * 
	 * @see Datalink#isAircraftArmed()
	 */
	@Override
	public synchronized boolean isAircraftArmed() {
		return this.isArmed;
	}
	
	/**
	 * Uploads a mission flight path to the aircraft connected via this
	 * simulated datalink.
	 * 
	 * @param mission the mission flight path to be uploaded
	 *
	 * @see Datalink#uploadMission(Path)
	 */
	@Override
	public synchronized void uploadMission(Path mission) {
		super.uploadMission(mission);
		
		if (!this.isAirborne) {
			// upload mission before flight
			this.flightPath = new Path(mission.getPositions());
			this.positionIndex = 0;
			this.positionIterator = this.flightPath.getPositions().iterator();
			if (this.positionIterator.hasNext()) {
				this.homePosition = this.positionIterator.next();
				this.lastPosition = this.homePosition;
				this.nextPosition = this.homePosition;
				this.iris.moveTo(this.homePosition);
			}
		} else {
			// upload mission during flight
			this.lastPosition = this.getAircraftPosition();
			this.nextPosition = this.lastPosition;
			this.flightPath = new Path(mission.getPositions());
			this.positionIndex = -1;
			this.positionIterator = this.flightPath.getPositions().iterator();
		}
		
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Downloads a mission flight path from the aircraft connected via this
	 * simulated datalink.
	 * 
	 * @param cached indicates whether or not the mission cache is used
	 * 
	 * @return the downloaded mission flight path
	 * 
	 * @see Datalink#downloadMission(boolean)
	 */
	@Override
	public synchronized Path downloadMission(boolean cached) {
		return cached ? super.downloadMission(cached) : this.flightPath;
	}
	
	/**
	 * Gets the next position of the mission flight path from the aircraft
	 * connected via this datalink.
	 * 
	 * @return the next position of the mission flight path
	 * 
	 * @see Datalink#getNextMissionPosition()
	 */
	@Override
	public synchronized Position getNextMissionPosition() {
		return this.nextPosition;
	}
	
	/**
	 * Gets the index of the next position of the mission flight path from
	 * the aircraft connected via this simulated datalink.
	 * 
	 * @return the index of the next position of the mission flight path
	 * 
	 * @see Datalink#getNextMissionPositionIndex()
	 */
	@Override
	public synchronized int getNextMissionPositionIndex() {
		return this.positionIndex;
	}
	
	/**
	 * Initiates a take-off for the aircraft connected via this simulated
	 * datalink.
	 * 
	 * @see Datalink#takeOff()
	 */
	@Override
	public synchronized void takeOff() {
		if (!this.isAirborne()) {
			this.setAircraftMode(SimulatedDatalink.MODE_AUTO);
			this.eto = ZonedDateTime.now(ZoneId.of("UTC"));
			this.ete = Duration.ZERO;
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			this.isAirborne = true;
		}
	}
	
	/**
	 * Initiates a landing for the aircraft connected via this simulated
	 * datalink.
	 * 
	 * @see Datalink#land()
	 */
	@Override
	public synchronized void land() {
		if (this.isAirborne) {
			this.setAircraftMode(SimulatedDatalink.MODE_LAND);
			this.nextPosition = this.iris.getReferencePosition();
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			this.isAirborne = false;
		}
	}
	
	/**
	 * Initiates a return to and landing at the launch position for the
	 * aircraft connected via this simulated datalink.
	 * 
	 * @see Datalink#returnToLaunch()
	 */
	@Override
	public synchronized void returnToLaunch() {
		if (this.isAirborne()) {
			this.setAircraftMode(SimulatedDatalink.MODE_RTL);
			Path rtl = new Path(this.getAircraftPosition(), this.homePosition);
			this.uploadMission(rtl);
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	/**
	 * Determines whether or not the aircraft connected via this simulated
	 * datalink is airborne.
	 * 
	 * @return true if the aircraft is airborne, false otherwise
	 * 
	 * @see Datalink#isAirborne()
	 */
	@Override
	public synchronized boolean isAirborne() {
		return this.isAirborne;
	}
	
	/**
	 * Gets the maximum track error of this simulated datalink.
	 * 
	 * @return the maximum track error of this simulated datalink
	 */
	public AircraftTrackError getMaxTrackError() {
		return this.maxTrackError;
	}
	
	/**
	 * Sets the maximum track error of this simulated datalink.
	 * 
	 * @param maxTrackError the maximum track error to be set
	 */
	public void setMaxTrackError(AircraftTrackError maxTrackError) {
		this.maxTrackError = maxTrackError;
	}
	
	/**
	 * Gets the error probability of this simulated datalink.
	 * 
	 * @return the error probability of this simulated datalink
	 */
	public float getErrorProbablilty() {
		return this.errorProbability;
	}
	
	/**
	 * Sets the error probablity of this simulated datalink.
	 * 
	 * @param errorProbablity the error probablity to be set
	 */
	public void setErrorProbablity(float errorProbablity) {
		this.errorProbability = errorProbablity;
	}
	
	/**
	 * Determines whether or not this simulated datalink matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this simulated datalink matches the specification,
	 *         false otherwise
	 * 
	 * @see Datalink#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof SimulatedDatalinkProperties)) {
			SimulatedDatalinkProperties properties =
					(SimulatedDatalinkProperties) specification.getProperties();
			matches = (this.maxTrackError.getCrossTrackError() == properties.getMaxCrossTrackError())
					&& (this.maxTrackError.getTimingError().equals(properties.getMaxTimingError()))
					&& (this.errorProbability == properties.getErrorProbability());
		}
		
		return matches;
	}
	
	/**
	 * Updates this simulated datalink according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this simulated datalink has been updated,
	 *         false otherwise
	 * 
	 * @see Datalink#update(Specification)
	 */
	@Override
	public boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof SimulatedDatalinkProperties)) {
			SimulatedDatalinkProperties properties =
					(SimulatedDatalinkProperties) specification.getProperties();
			this.getMaxTrackError().setCrossTrackError(properties.getMaxCrossTrackError());
			this.getMaxTrackError().setTimingError(Duration.ofSeconds(properties.getMaxTimingError()));
			this.setErrorProbablity(properties.getErrorProbability());
		}
		
		return updated;
	}
	
}
