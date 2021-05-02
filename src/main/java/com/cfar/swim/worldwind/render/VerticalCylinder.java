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
package com.cfar.swim.worldwind.render;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Cylinder;

/**
 * Realizes a renderable vertical cylinder.
 * 
 * @author Stephan Heinemann
 *
 */
public class VerticalCylinder extends Cylinder {
	
	/**
	 * Constructs a vertical cylinder with a specified center position, height
	 * and radius.
	 * 
	 * @param centerPosition the center position of this vertical cylinder
	 * @param height the height in meters of this vertical cylinder
	 * @param radius the radius in meters of this vertical cylinder
	 * 
	 * @see Cylinder#Cylinder(Position, double, double)
	 */
	public VerticalCylinder(Position centerPosition, double height, double radius) {
		super(centerPosition, height, radius);
	}
	
	/**
	 * Sets the radius of this vertical cylinder.
	 * 
	 * @param eastWestRadius the radius of this vertical cylinder in meters
	 */
	@Override
	public void setEastWestRadius(double eastWestRadius) {
		this.eastWestRadius = eastWestRadius;
		this.northSouthRadius = eastWestRadius;
	}
	
	/**
	 * Sets the radius of this vertical cylinder.
	 * 
	 * @param northSouthRadius the radius of this vertical cylinder in meters
	 */
	@Override
	public void setNorthSouthRadius(double northSouthRadius) {
		this.northSouthRadius = northSouthRadius;
		this.eastWestRadius = northSouthRadius;
	}
	
	/**
	 * Sets the radius of this vertical cylinder.
	 * 
	 * @param radius the radius of this vertical cylinder in meters
	 */
	public void setRadius(double radius) {
		this.setEastWestRadius(radius);
	}
	
	/**
	 * Gets the radius of this vertical cylinder.
	 * 
	 * @return the radius of this vertical cylinder in meters
	 */
	public double getRadius() {
		return this.eastWestRadius;
	}
	
	/**
	 * Sets the height of this vertical cylinder.
	 * 
	 * @param height the height of this vertical cylinder in meters
	 */
	public void setHeight(double height) {
		this.verticalRadius = height * 0.5d;
	}
	
	/**
	 * Gets the height of this vertical cylinder.
	 * 
	 * @return the height of this vertical cylinder in meters.
	 */
	public double getHeight() {
		return (this.verticalRadius * 2d);
	}
	
	/**
	 * Throws an unsupported operation exception.
	 * 
	 * @see UnsupportedOperationException
	 */
	@Override
	public void setHeading(Angle heading) {
		throw new UnsupportedOperationException();
	}
	
	/**
	 * Throws an unsupported operation exception.
	 * 
	 * @see UnsupportedOperationException
	 */
	@Override
	public void setTilt(Angle tilt) {
		throw new UnsupportedOperationException();
	}
	
	/**
	 * Throws an unspported operation exception.
	 * 
	 * @see UnsupportedOperationException
	 */
	@Override
	public void setRoll(Angle roll) {
		throw new UnsupportedOperationException();
	}
	
	/**
	 * Converts this vertical cylinder to a geometric cylinder.
	 * 
	 * @param globe the globe to be used for the conversion
	 * @return the geometric cylinder
	 */
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
