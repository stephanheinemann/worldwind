package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.time.Duration;

import org.junit.Test;

import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Earth;

public class AircraftTest {

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
		
		// climb 50 meters
		goal = new Position(start, start.getElevation() + 50d);
		duration = iris.getCapabilities().getEstimatedDuration(start, goal, earth);
		assertEquals(5d, duration.getSeconds(), PrecisionDouble.EPSILON);
		
		// descent 100 meters
		goal = new Position(start, start.getElevation() - 100d);
		duration = iris.getCapabilities().getEstimatedDuration(start, goal, earth);
		assertEquals(10d, duration.getSeconds(), PrecisionDouble.EPSILON);
		
		// descent 1000 meters and level off
		goal = new Position(location, start.getElevation() - 1000d);
		duration = iris.getCapabilities().getEstimatedDuration(start, goal, earth);
		assertNotEquals(totalSeconds, duration.getSeconds(), PrecisionDouble.EPSILON);
	}

}
