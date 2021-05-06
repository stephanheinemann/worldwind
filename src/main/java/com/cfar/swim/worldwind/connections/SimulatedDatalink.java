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

import java.time.ZonedDateTime;
import java.util.Iterator;

import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;

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
	
	/** the estimated time on a leg of the planned flight path */
	private double ete = 0d;
	
	/** the actual time on a leg of the planned flight path */
	private double ate = 0d;
	
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
	
	/**
	 * Constructs a new simulated datatlink.
	 */
	public SimulatedDatalink() {
		this.getAircraftTrack().setName("Simulated Iris");
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
	
	/**
	 * Gets the aircraft status via this simulated datalink.
	 * 
	 * @return the aircraft status obtained via this simulated datalink
	 * 
	 * @see Datalink#getAircraftStatus()
	 */
	@Override
	public String getAircraftStatus() {
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
	public String getAircraftMode() {
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
	public void setAircraftMode(String aircraftMode) {
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
	public Angle getAircraftHeading() {
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
	public Position getAircraftPosition() {
		Position currentPosition = this.iris.getReferencePosition();
		
		if ((null != this.flightPath) && this.isAirborne) {
			if (this.ate >= this.ete) {
				this.lastPosition = this.nextPosition;
				if (this.positionIterator.hasNext()) {
					this.positionIndex++;
					this.nextPosition = this.positionIterator.next();
					this.ete = this.iris.getCapabilities().getEstimatedDuration(
							lastPosition, nextPosition, this.globe).getSeconds();
					this.ate = 0d;
				}
			}
			double slice = ((double) this.getDownlinkPeriod() / 1000d);
			this.ate += slice;
			if (this.ate > this.ete) this.ate = this.ete;
			double ratio = this.ate / this.ete;
			currentPosition = Position.interpolate(ratio, lastPosition, nextPosition);
			this.iris.moveTo(currentPosition);
		}
		
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
	public AircraftTrackPoint getAircraftTrackPoint() {
		AircraftTrackPoint trackPoint = new AircraftTrackPoint(this.getAircraftPosition());
		trackPoint.setPitch(this.getAircraftPitch());
		trackPoint.setBank(this.getAircraftBank());
		trackPoint.setHeading(this.getAircraftHeading());
		trackPoint.setAto(ZonedDateTime.now());
		return trackPoint;
	}
	
	/**
	 * Enables the aircraft safety via this simulated datalink.
	 * 
	 * @see Datalink#enableAircraftSafety()
	 */
	@Override
	public void enableAircraftSafety() {
		this.isAircraftSafetyEnabled = true;
	}
	
	/**
	 * Disables the aircraft safety via this simulated datalink.
	 * 
	 * @see Datalink#disableAircraftSafety()
	 */
	@Override
	public void disableAircraftSafety() {
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
	public boolean isAircraftSafetyEnabled() {
		return this.isAircraftSafetyEnabled;
	}
	
	/**
	 * Arms the aircraft via this simulated datalink.
	 * 
	 * @see Datalink#armAircraft()
	 */
	@Override
	public void armAircraft() {
		this.isArmed = true;
	}
	
	/**
	 * Disarms the aircraft via this simulated datalink.
	 * 
	 * @see Datalink#disarmAircraft()
	 */
	@Override
	public void disarmAircraft() {
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
	public boolean isAircraftArmed() {
		return this.isArmed;
	}
	
	/**
	 * Uploads a mission flight path to the aircraft connected via this
	 * simulated datalink.
	 * 
	 * @param path the mission flight path to be uploaded
	 *
	 * @see Datalink#uploadMission(Path)
	 */
	@Override
	public void uploadMission(Path flightPath) {
		this.flightPath = flightPath;
		this.ate = 0d;
		this.ete = 0d;
		this.positionIndex = 0;
		this.positionIterator = this.flightPath.getPositions().iterator();
		if (this.positionIterator.hasNext()) {
			this.homePosition = this.positionIterator.next();
			this.lastPosition = this.homePosition;
			this.nextPosition = this.homePosition;
			this.iris.moveTo(this.homePosition);
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
	 * @return the downloaded mission flight path
	 * 
	 * @see Datalink#downloadMission()
	 */
	@Override
	public Path downloadMission() {
		return this.flightPath;
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
	public Position getNextMissionPosition() {
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
	public int getNextMissionPositionIndex() {
		return this.positionIndex;
	}
	
	/**
	 * Initiates a take-off for the aircraft connected via this simulated
	 * datalink.
	 * 
	 * @see Datalink#takeOff()
	 */
	@Override
	public void takeOff() {
		this.setAircraftMode(SimulatedDatalink.MODE_AUTO);
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		this.isAirborne = true;
	}
	
	/**
	 * Initiates a landing for the aircraft connected via this simulated
	 * datalink.
	 * 
	 * @see Datalink#land()
	 */
	@Override
	public void land() {
		this.setAircraftMode(SimulatedDatalink.MODE_LAND);
		this.nextPosition = this.iris.getReferencePosition();
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		this.isAirborne = false;
	}
	
	/**
	 * Initiates a return to and landing at the launch position for the
	 * aircraft connected via this simulated datalink.
	 * 
	 * @see Datalink#returnToLaunch()
	 */
	@Override
	public void returnToLaunch() {
		this.setAircraftMode(SimulatedDatalink.MODE_RTL);
		this.nextPosition = this.homePosition;
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		this.isAirborne = false;
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
	public boolean isAirborne() {
		return this.isAirborne;
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
	public final boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		matches &= (specification.getId().equals(Specification.CONNECTION_DATALINK_SIMULATED_ID));
		return matches;
	}
	
}
