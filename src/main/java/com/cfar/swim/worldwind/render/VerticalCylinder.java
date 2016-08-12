package com.cfar.swim.worldwind.render;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Cylinder;

public class VerticalCylinder extends Cylinder {

	public VerticalCylinder(Position centerPosition, double height, double radius) {
		super(centerPosition, height, radius);
	}
	
	@Override
	public void setEastWestRadius(double eastWestRadius) {
		this.eastWestRadius = eastWestRadius;
		this.northSouthRadius = eastWestRadius;
	}
	
	@Override
	public void setNorthSouthRadius(double northSouthRadius) {
		this.northSouthRadius = northSouthRadius;
		this.eastWestRadius = northSouthRadius;
	}
	
	@Override
	public void setHeading(Angle heading) {
		throw new UnsupportedOperationException();
	}
	
	@Override
	public void setTilt(Angle tilt) {
		throw new UnsupportedOperationException();
	}
	
	@Override
	public void setRoll(Angle roll) {
		throw new UnsupportedOperationException();
	}
	
	public gov.nasa.worldwind.geom.Cylinder toGeometricCylinder(Globe globe) {
		Position center = this.getCenterPosition();
		Angle latitude = center.getLatitude();
		Angle longitude = center.getLongitude();
		double elevation = center.getElevation();
		double radius = this.getEastWestRadius();
		
		Position bcp = new Position(latitude, longitude, elevation - this.getVerticalRadius());
		Position tcp = new Position(latitude, longitude, elevation + this.getVerticalRadius());
		Vec4 bottomCenter = globe.computePointFromPosition(bcp);
		Vec4 topCenter = globe.computePointFromPosition(tcp);
		
		return new gov.nasa.worldwind.geom.Cylinder(bottomCenter, topCenter, radius);
	}
	
}
