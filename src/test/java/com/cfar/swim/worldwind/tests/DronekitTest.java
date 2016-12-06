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
package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.cfar.swim.worldwind.connections.AircraftConnection;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.AircraftConnectionFactory;
import com.cfar.swim.worldwind.registries.connections.DronekitConnectionProperties;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;

public class DronekitTest {

	@Test
	public void testConnection() throws InterruptedException {
		Specification<AircraftConnection> aircraftSpecification = new Specification<>(Specification.CONNECTION_AIRCRAFT_DRONEKIT, new DronekitConnectionProperties());
		DronekitConnectionProperties properties = (DronekitConnectionProperties) aircraftSpecification.getProperties();
		properties.setHost("206.87.166.221");
		properties.setPort(50051);
		AircraftConnectionFactory aircraftConnectionFactory = new AircraftConnectionFactory();
		AircraftConnection connection = aircraftConnectionFactory.createInstance(aircraftSpecification);
		
		// connect
		connection.connect();
		assertTrue(connection.isConnected());
		
		Position first = new Position(Angle.POS90, Angle.POS180, 9500d);
		Position second = new Position(Angle.NEG90, Angle.NEG180, 9700d);
		assertEquals(second.getAltitude(), 9700d, 0.1d);
		
		Path path = new Path(first, second);
		connection.uploadPath(path);
		
		while(!connection.getAircraftMode().equals("AUTO")) {
			System.out.println(connection.getAircraftMode());
			Thread.sleep(1000);
		}
		
		// set mode
		connection.setAircraftMode("STABILIZE");
		System.out.println(connection.getAircraftPosition());
		
		// set safety
		connection.disableAircraftSafety();
		while (connection.isAircraftSafetyEnabled()) {
			Thread.sleep(1000);
		}
		assertFalse(connection.isAircraftSafetyEnabled());
		
		// arm
		connection.armAircraft();
		while (!connection.isAircraftArmed()) {
			Thread.sleep(1000);
		}
		assertTrue(connection.isAircraftArmed());
		
		// cleanup
		connection.disarmAircraft();
		connection.enableAircraftSafety();
		connection.disconnect();
		assertFalse(connection.isConnected());
	}

}
