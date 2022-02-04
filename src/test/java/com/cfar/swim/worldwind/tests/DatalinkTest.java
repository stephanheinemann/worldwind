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
package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Ignore;
import org.junit.Test;

import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.connections.DronekitDatalink;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.DatalinkFactory;
import com.cfar.swim.worldwind.registries.connections.DronekitDatalinkProperties;
import com.cfar.swim.worldwind.registries.connections.MavlinkDatalinkProperties;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.util.Logging;
import io.dronefleet.mavlink.common.MavMode;
import io.dronefleet.mavlink.common.MavModeFlag;
import io.dronefleet.mavlink.util.EnumValue;

/**
 * Performs datalink tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class DatalinkTest {

	/**
	 * Tests the dronekit datalink connection.
	 * 
	 * @throws InterruptedException if the datalink thread gets interrupted unexpectedly
	 */
	@Ignore
	@Test
	public void testDronekit() throws InterruptedException {
		Specification<Datalink> datalinkSpecification = new Specification<>(
				Specification.CONNECTION_DATALINK_DRONEKIT_ID,
				new DronekitDatalinkProperties());
		DronekitDatalinkProperties properties = (DronekitDatalinkProperties)
				datalinkSpecification.getProperties();
		properties.setHost("206.87.166.221");
		properties.setPort(50051);
		DatalinkFactory datalinkFactory = new DatalinkFactory();
		datalinkFactory.setSpecification(datalinkSpecification);
		Datalink datalink = datalinkFactory.createInstance();
		
		// connect
		datalink.connect();
		assertTrue(datalink.isConnected());
		
		Position first = new Position(Angle.POS90, Angle.POS180, 9500d);
		Position second = new Position(Angle.NEG90, Angle.NEG180, 9700d);
		assertEquals(second.getAltitude(), 9700d, 0.1d);
		
		Path path = new Path(first, second);
		datalink.uploadMission(path);
		
		while(!datalink.getAircraftMode().equals(DronekitDatalink.MODE_AUTO)) {
			Logging.logger().info(datalink.getAircraftMode());
			Thread.sleep(1000);
		}
		
		// set mode
		datalink.setAircraftMode(DronekitDatalink.MODE_STABILIZE);
		Logging.logger().info(datalink.getAircraftPosition().toString());
		
		// set safety
		datalink.disableAircraftSafety();
		while (datalink.isAircraftSafetyEnabled()) {
			Thread.sleep(1000);
		}
		assertFalse(datalink.isAircraftSafetyEnabled());
		
		// arm
		datalink.armAircraft();
		while (!datalink.isAircraftArmed()) {
			Thread.sleep(1000);
		}
		assertTrue(datalink.isAircraftArmed());
		
		// cleanup
		datalink.disarmAircraft();
		datalink.enableAircraftSafety();
		datalink.disconnect();
		assertFalse(datalink.isConnected());
	}
	
	/**
	 * Tests the mavlink datalink connection.
	 * 
	 * @throws InterruptedException 
	 */
	//@Ignore
	@Test
	public void testMavlink() throws InterruptedException {
		/*int a = 29;
		int b = 157;
		EnumValue<MavMode> modeA = EnumValue.create(a);
		EnumValue<MavMode> modeB = EnumValue.create(b);
		
		modeA.flagsEnabled(MavMode.MAV_MODE_AUTO_ARMED);
		modeA.flagsEnabled(MavMode.MAV_MODE_AUTO_DISARMED);
		modeA.flagsEnabled(MavMode.MAV_MODE_GUIDED_ARMED);
		modeA.flagsEnabled(MavMode.MAV_MODE_GUIDED_DISARMED);
		modeA.flagsEnabled(MavMode.MAV_MODE_MANUAL_ARMED);
		modeA.flagsEnabled(MavMode.MAV_MODE_MANUAL_DISARMED);
		modeA.flagsEnabled(MavMode.MAV_MODE_PREFLIGHT);
		modeA.flagsEnabled(MavMode.MAV_MODE_STABILIZE_ARMED);
		modeA.flagsEnabled(MavMode.MAV_MODE_TEST_ARMED);
		modeA.flagsEnabled(MavMode.MAV_MODE_TEST_DISARMED);
		
		modeB.flagsEnabled(MavMode.MAV_MODE_AUTO_ARMED);
		modeB.flagsEnabled(MavMode.MAV_MODE_AUTO_DISARMED);
		modeB.flagsEnabled(MavMode.MAV_MODE_GUIDED_ARMED);
		modeB.flagsEnabled(MavMode.MAV_MODE_GUIDED_DISARMED);
		modeB.flagsEnabled(MavMode.MAV_MODE_MANUAL_ARMED);
		modeB.flagsEnabled(MavMode.MAV_MODE_MANUAL_DISARMED);
		modeB.flagsEnabled(MavMode.MAV_MODE_PREFLIGHT);
		modeB.flagsEnabled(MavMode.MAV_MODE_STABILIZE_ARMED);
		modeB.flagsEnabled(MavMode.MAV_MODE_TEST_ARMED);
		modeB.flagsEnabled(MavMode.MAV_MODE_TEST_DISARMED);
		*/
		
		Specification<Datalink> datalinkSpecification = new Specification<>(
				Specification.CONNECTION_DATALINK_MAVLINK_ID,
				new MavlinkDatalinkProperties());
		MavlinkDatalinkProperties properties = (MavlinkDatalinkProperties)
				datalinkSpecification.getProperties();
		//properties.setHost("172.16.104.128");
		//properties.setPort(14550);
		// target and source identifiers
		DatalinkFactory datalinkFactory = new DatalinkFactory();
		datalinkFactory.setSpecification(datalinkSpecification);
		Datalink datalink = datalinkFactory.createInstance();
		
		assertFalse(datalink.isConnected());
		datalink.connect();
		assertTrue(datalink.isConnected());
		
		
		//assertFalse(datalink.isAircraftArmed());
		//datalink.disableAircraftSafety();
		//datalink.armAircraft();
		//assertTrue(datalink.isAircraftArmed());
		//assertTrue(datalink.isAircraftSafetyEnabled());
		//datalink.disableAircraftSafety();
		//assertFalse(datalink.isAircraftSafetyEnabled());
		
		//Thread.sleep(5000);
		
		//datalink.enableAircraftSafety();
		//assertTrue(datalink.isAircraftSafetyEnabled());
		
		//Trajectory trajectory = new Trajectory(new Waypoint(Position.ZERO), new Waypoint(Position.ZERO));
		//datalink.uploadMission(trajectory);
		Path mission = datalink.downloadMission(false);
		mission.getPositions().forEach(p -> System.out.println(p));
		//datalink.downloadMission(false);
		
		
		datalink.disconnect();
		assertFalse(datalink.isConnected());
	}
	
}
