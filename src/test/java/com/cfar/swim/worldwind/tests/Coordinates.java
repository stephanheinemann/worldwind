package com.cfar.swim.worldwind.tests;

import java.time.LocalDate;
import java.time.LocalTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.Random;

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
		
		//Start and Goal positions
		Position positionStart = new Position(Angle.fromDegrees(20), Angle.fromDegrees(80), 40);
		System.out.println("Start Position: \t"+positionStart);
		Position positionGoal = new Position(Angle.fromDegrees(20.01), Angle.fromDegrees(80.01), 40);
		System.out.println("Goal Position:  \t"+positionGoal);
		
		double distance = 1.5*samplingEnv.getDistance(origin, destination);
		System.out.println("Min distance (2c) =\t"+samplingEnv.getDistance(origin, destination)+
				"\tMax distance (2a) =\t"+distance);
		for(int i=0; i<100; i++) {
			Position positionRand = samplingEnv.samplePositionEllipsoid(origin, destination, distance);
			double distanceSum = samplingEnv.getDistance(origin, positionRand)+samplingEnv.getDistance(positionRand, destination);
			boolean inside = samplingEnv.contains(positionRand);
			System.out.println("Rand Position:  \t"+positionRand+"\tdS+dG = "+distanceSum+"\tInside? "+inside);
		}
		
		/*
		//Middle Position from start and goal
		Position positionM = CoordinateTransformations.middlePosition(positionStart, positionGoal, globe);
		System.out.println("Middle Position:\t"+positionM);
		
		//Transformation of start and goal to ENU centered at middle position
		Vec4 enuStart = CoordinateTransformations.llh2enu(positionM, positionStart, globe);
		System.out.println("Start ENU: \t"+enuStart);
		Vec4 enuGoal = CoordinateTransformations.llh2enu(positionM, positionGoal, globe);
		System.out.println("Goal ENU:  \t"+enuGoal);
		
		//Computation of AER of Start and Goal in middle position ENU frame
		Vec4 aerStart = CoordinateTransformations.enu2aer(enuStart);
		System.out.println("Start AER: \t"+aerStart);
		Vec4 aerGoal = CoordinateTransformations.enu2aer(enuGoal);
		System.out.println("Goal AER:  \t"+aerGoal);

		//Rotation of frame to align axis y as pointing from Start to Goal
		Vec4 pointA, pointB;
		double angleZ = Math.PI/2 -aerGoal.x, angleX = aerGoal.y;
		pointA = CoordinateTransformations.rotationZ(enuStart, angleZ);
		pointA = CoordinateTransformations.rotationX(pointA, angleX);
		System.out.println("Start point: \t"+pointA);
		pointB = CoordinateTransformations.rotationZ(enuGoal, angleZ);
		pointB = CoordinateTransformations.rotationX(pointB, angleX);
		System.out.println("Goal point:  \t"+pointB);
		
		//REVERSE TRANSFORMATION
		//Rotation of frame from axis y pointing Start->Goal to ENU at middle posiiton 
		Vec4 enuAR, enuBR;
		double angleZR = -angleZ, angleXR = -angleX;
		enuAR = CoordinateTransformations.rotationZ(pointA, angleZR);
		enuAR = CoordinateTransformations.rotationX(enuAR, angleXR);
		System.out.println("Start ENUR: \t"+enuAR);
		enuBR = CoordinateTransformations.rotationZ(pointB, angleZR);
		enuBR = CoordinateTransformations.rotationX(enuBR, angleXR);
		System.out.println("Goal ENUR:  \t"+enuBR);
		
		//Transformation of start and goal from ENU centered at middle position
		Position positionStartR = CoordinateTransformations.enu2llh(positionM, enuAR, globe);
		System.out.println("Start PositionR: \t"+positionStartR);
		Position positionGoalR = CoordinateTransformations.enu2llh(positionM, enuBR, globe);
		System.out.println("Goal PositionR:  \t"+positionGoalR);

		
		//TESTING HOW TO SAMPLE FROM AN ELLIPSIS
		double distance = 1.1*samplingEnv.getDistance(positionStart, positionGoal);
		System.out.println("distance = "+distance);
		
		// Sample random angle and distance
		double r = 0d + new Random().nextDouble() * 1d;
		double theta = 0d + new Random().nextDouble() * Math.PI;
		double phi = 0d + new Random().nextDouble() * 2*Math.PI;
		Vec4 pointRand = CoordinateTransformations.polar2cartesian(phi, theta, r);
		System.out.println("Rand point:  \t"+pointRand);
		
		// Ellipsoid parameters
		double a = distance/2d;
		double c = samplingEnv.getDistance(positionStart, positionGoal)/2d;
		double b = Math.sqrt(a*a - c*c);
		System.out.println("C from points= "+enuStart.distanceTo3(enuGoal)+
				"\tC from positions= "+samplingEnv.getDistance(positionStart, positionGoal));
		System.out.println("a = "+a+"\tb  = "+b+"\tc = "+c);
		
		//Transform from sphere to ellipsoid
		Vec4 pointRandE = new Vec4( pointRand.x * b, pointRand.y * a, pointRand.z * b);
		System.out.println("Point RandE: "+pointRandE);
		
		//Transform to llh
		Vec4 enuRand = CoordinateTransformations.rotationZ(pointRandE, angleZR);
		enuRand = CoordinateTransformations.rotationX(enuRand, angleXR);
		System.out.println("Rand ENUR: \t"+enuRand);
		Position positionRandR = CoordinateTransformations.enu2llh(positionM, enuRand, globe);
		System.out.println("Rand PositionR: \t"+positionRandR);
		*/
				
		
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
