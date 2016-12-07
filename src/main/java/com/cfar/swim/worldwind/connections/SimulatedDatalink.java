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

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;

public class SimulatedDatalink extends Datalink {
	
	private boolean isConnected = false;
	private boolean isArmed = false;
	private boolean isAircraftSafetyEnabled = true;
	private String aircraftMode = "UNKNOWN";
	private Position position = new Position(Angle.ZERO, Angle.ZERO, 10000d);
	
	@Override
	public void connect() {
		this.isConnected = true;
	}
	
	@Override
	public void disconnect() {
		this.isConnected = false;
	}
	
	@Override
	public boolean isConnected() {
		return this.isConnected;
	}
	
	@Override
	public String getAircraftMode() {
		return this.aircraftMode;
	}

	@Override
	public void setAircraftMode(String aircraftMode) {
		this.aircraftMode = aircraftMode;
	}

	@Override
	public Position getAircraftPosition() {
		Position delta = new Position(Angle.fromDegrees(1d), Angle.fromDegrees(1d), 0d);
		this.position = this.position.add(delta);
		return this.position;
	}

	@Override
	public void armAircraft() {
		this.isArmed = true;
	}

	@Override
	public void disarmAircraft() {
		this.isArmed = false;
	}

	@Override
	public boolean isAircraftArmed() {
		return this.isArmed;
	}

	@Override
	public void uploadFlightPath(Path path) {
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void enableAircraftSafety() {
		this.isAircraftSafetyEnabled = true;
	}

	@Override
	public void disableAircraftSafety() {
		this.isAircraftSafetyEnabled = false;
	}

	@Override
	public boolean isAircraftSafetyEnabled() {
		return this.isAircraftSafetyEnabled;
	}

	@Override
	public void takeOff() {
		this.setAircraftMode("GUIDED");
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void land() {
		this.setAircraftMode("LAND");
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void returnToLaunch() {
		this.setAircraftMode("RTL");
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	// TODO: a simulated connection shall aggregate a runnable
	// simulating a selected aircraft with its telemetry

}
