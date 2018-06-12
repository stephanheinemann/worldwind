package com.cfar.swim.worldwind.tests;

import org.junit.Test;

import com.cfar.swim.worldwind.geom.CoordinateTransformations;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;

public class Coordinates {
	
	Globe globe = new Earth();

	@Test
	public void Main() {
		System.out.println("\n");
		
		Position positionNear = new Position(Angle.fromDegrees(20), Angle.fromDegrees(80), 0);
		System.out.println("Reference Position:\t"+positionNear);
		
		Position positionRand = new Position(positionNear, 100);
		System.out.println("Other Position:    \t"+positionRand);
		
		Vec4 pointENU = CoordinateTransformations.ecef2enu(positionNear, positionRand, globe);
		System.out.println("pointENU: "+pointENU);
		
		Position positionNew = CoordinateTransformations.enu2ecef(positionNear, pointENU, globe);
		System.out.println("PositionNew: " + positionNew);
		
		System.out.println("\n");
	}
	
}
