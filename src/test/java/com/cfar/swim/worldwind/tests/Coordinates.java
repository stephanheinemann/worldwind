package com.cfar.swim.worldwind.tests;

import java.time.LocalDate;
import java.time.LocalTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;

import org.junit.Test;

import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.CoordinateTransformations;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;

public class Coordinates {
	
	Iris iris;
	A320 a320;
	Globe globe;
	SamplingEnvironment samplingEnv;
	Position origin, destination;
	ZonedDateTime etd;

	@Test
	public void Main() {
		System.out.println("\n");
		this.setScenario();
		
		Position positionStart = new Position(Angle.fromDegrees(20), Angle.fromDegrees(80), 40);
		System.out.println("Start Position: \t"+positionStart);
		
		Position positionGoal = new Position(Angle.fromDegrees(20.01), Angle.fromDegrees(80.01), 40);
		System.out.println("Goal Position:  \t"+positionGoal);
		
		Position positionM = CoordinateTransformations.middlePosition(positionStart, positionGoal, globe);
		System.out.println("Middle Position:\t"+positionM);
		
		System.out.println("\n");
	}
	
	public void setScenario() {
		// Create box area in globe
		globe = new Earth();
		Sector cadboroBay = new Sector(
				Angle.fromDegrees(48.44),
				Angle.fromDegrees(48.46),
				Angle.fromDegrees(-123.29),
				Angle.fromDegrees(-123.27));
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, cadboroBay, 0.0, 1500.0);

		// Create environment from box
		samplingEnv = new SamplingEnvironment(new Box(boxNASA));
		samplingEnv.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(48.445, -123.285, 10);
		destination = Position.fromDegrees(48.455, -123.275, 100);
		etd = ZonedDateTime.of(LocalDate.of(2018, 5, 1), LocalTime.of(22, 0), ZoneId.of("UTC"));
		iris = new Iris(origin, 500, CombatIdentification.FRIEND);
		a320 = new A320(origin, 5000, CombatIdentification.FRIEND);
	}
}
