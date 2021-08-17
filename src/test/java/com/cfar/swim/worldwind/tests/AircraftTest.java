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
import static org.junit.Assert.assertNotEquals;

import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;

import org.junit.Test;

import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.render.Path;

/**
 * Performs aircraft tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class AircraftTest {

	/**
	 * Tests aircraft capabilities.
	 */
	@Test
	public void testCapabilities() {
		Earth earth = new Earth();
		Position start = new Position(Angle.ZERO, Angle.ZERO, 1000d);
		Iris iris = new Iris(start, 1000d, CombatIdentification.FRIEND);
		
		// level flight along 60 nautical miles
		LatLon location = new LatLon(Angle.fromDegrees(0d), Angle.fromDegrees(1d));
		Position goal = start.add(new Position(location, 0d));
		Duration duration = iris.getCapabilities().getEstimatedDuration(start, goal, earth);
		double totalSeconds = ((double) duration.getSeconds()) + ((double) duration.getNano() * 10E-9);
		assertEquals(iris.getCapabilities().getCruiseSpeed(), 111120d / totalSeconds, 0.1d);
		
		Path leg = new Path(start, goal);
		ZonedDateTime etd = ZonedDateTime.of(2016, 10, 13, 21, 31, 0, 0, ZoneId.of("UTC"));
		ZonedDateTime eta = iris.getCapabilities().getEstimatedTime(leg, earth, etd);
		assertEquals(etd.plusSeconds(duration.getSeconds()).plusNanos(duration.getNano()), eta);
		
		// climb 50 meters
		goal = new Position(start, start.getElevation() + 50d);
		duration = iris.getCapabilities().getEstimatedDuration(start, goal, earth);
		assertEquals(5d, duration.getSeconds() + (duration.getNano() * 10E-9), PrecisionDouble.EPSILON);
		
		// descent 100 meters
		goal = new Position(start, start.getElevation() - 100d);
		duration = iris.getCapabilities().getEstimatedDuration(start, goal, earth);
		assertEquals(10d, duration.getSeconds() + (duration.getNano() * 10E-9), PrecisionDouble.EPSILON);
		
		// descent 1000 meters and level off
		goal = new Position(location, start.getElevation() - 1000d);
		duration = iris.getCapabilities().getEstimatedDuration(start, goal, earth);
		assertNotEquals(totalSeconds, duration.getSeconds() + (duration.getNano() * 10E-9), PrecisionDouble.EPSILON);
	}
	
}
