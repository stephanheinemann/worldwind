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
package com.cfar.swim.worldwind.connections;

import java.util.Iterator;

import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
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
	
	/** the aircraft mode of this simulated datalink source */
	private String aircraftMode = "UNKNOWN";
	
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
	 * Indicates whether or not this simulated datalink is connected.
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
	 * Gets the aircraft mode via this simulated datalink.
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
	 * Indicates whether or not the aircraft connected via this simulated
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
	 * Uploads a flight path to the aircraft connected via this simulated
	 * datalink.
	 * 
	 * @param path the flight path to be uploaded
	 * 
	 * @see Datalink#uploadFlightPath(Path)
	 */
	@Override
	public void uploadFlightPath(Path flightPath) {
		this.flightPath = flightPath;
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
	 * Indicates whether or not the aircraft safety is enabled for the aircraft
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
	 * Initiates a take-off for the aircraft connected via this simulated
	 * datalink.
	 * 
	 * @see Datalink#takeOff()
	 */
	@Override
	public void takeOff() {
		this.setAircraftMode("AUTO");
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
		this.setAircraftMode("LAND");
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
		this.setAircraftMode("RTL");
		this.nextPosition = this.homePosition;
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		this.isAirborne = false;
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
		matches &= (specification.getId().equals(Specification.DATALINK_SIMULATED));
		return matches;
	}
	
}
