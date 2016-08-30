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
package com.cfar.swim.worldwind.data;

import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBElement;

import com.cfar.swim.worldwind.render.ObstacleCylinder;

import aero.aixm.AirspaceTimeSliceType;
import aero.aixm.AirspaceVolumeType;
import aero.aixm.CodeAirspaceDesignatorType;
import aero.aixm.CodeAirspaceType;
import aero.aixm.SurfaceType;
import aero.aixm.TextNameType;
import aero.aixm.ValDistanceVerticalType;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.AbstractSurfaceShape;
import gov.nasa.worldwind.render.RigidShape;
import gov.nasa.worldwind.render.SurfaceCircle;
import gov.nasa.worldwind.render.airspaces.Airspace;
import net.opengis.gml.AbstractSurfacePatchType;

public class AixmData {
	
	public static List<RigidShape> getRigidShapes(AirspaceVolumeType airspace) {
		List<RigidShape> rigidShapes = new ArrayList<RigidShape>();
		double lowerLimit = 0.0;
		double upperLimit = 0.0;
		
		// TODO: incorporate references
		if (null != airspace.getLowerLimit()) {
			lowerLimit = AixmData.getHeight(airspace.getLowerLimit().getValue());
		}
		
		if (null != airspace.getUpperLimit()) {
			upperLimit = AixmData.getHeight(airspace.getUpperLimit().getValue());
		}
			
		// TODO: implement other airspace volume elements
		List<? extends AbstractSurfaceShape> surfaceShapes =
				AixmData.getSurfaceShapes(airspace.getHorizontalProjection().getValue().getSurface().getValue());
		for (AbstractSurfaceShape surfaceShape : surfaceShapes) {
			if (surfaceShape instanceof SurfaceCircle) {
				if (upperLimit >= lowerLimit) {
					double height = upperLimit - lowerLimit;
					double altitude = lowerLimit + (height / 2.0);
					LatLon location = ((SurfaceCircle) surfaceShape).getCenter();
					double radius = ((SurfaceCircle) surfaceShape).getRadius();
					Position center = new Position(location, altitude);
					// TODO: previous reports may have indicated vertical limits that are to be used
					if (0 == height) {
						height++;
					}
					// TODO: previous reports may have indicated a radius that is to be used
					if (0 == radius) {
						radius++;
					}
					rigidShapes.add(new ObstacleCylinder(center, height, radius));
				}
			}
		}	
		
		return rigidShapes;
	}
	
	public static List<Airspace> getAirspaces(AirspaceVolumeType airspace) {
		List<Airspace> airspaces = new ArrayList<Airspace>();
		double lowerLimit = 0.0;
		double upperLimit = 0.0;
		
		// TODO: incorporate references
		if (null != airspace.getLowerLimit()) {
			lowerLimit = AixmData.getHeight(airspace.getLowerLimit().getValue());
		}
		
		if (null != airspace.getUpperLimit()) {
			upperLimit = AixmData.getHeight(airspace.getUpperLimit().getValue());
		}
			
		// TODO: implement other airspace volume elements
		List<? extends AbstractSurfaceShape> surfaceShapes =
				AixmData.getSurfaceShapes(airspace.getHorizontalProjection().getValue().getSurface().getValue());
		for (AbstractSurfaceShape surfaceShape : surfaceShapes) {
			if (surfaceShape instanceof SurfaceCircle) {
				if (upperLimit >= lowerLimit) {
					LatLon location = ((SurfaceCircle) surfaceShape).getCenter();
					double height = upperLimit - lowerLimit;
					double radius = ((SurfaceCircle) surfaceShape).getRadius();
					
					// NOTE: still required for geometric conversions
					// TODO: previous reports may have indicated vertical limits that are to be used
					if (0 == height) {
						upperLimit++;
					}
					// TODO: previous reports may have indicated a radius that is to be used
					if (0 == radius) {
						radius++;
					}
					airspaces.add(new com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder(location, lowerLimit, upperLimit, radius));
				}
			}
		}	
		
		return airspaces;
	}
	
	public static double getHeight(ValDistanceVerticalType altitude) {
		double alt = Double.parseDouble(altitude.getValue()); 
		return Conversions.toMeters(alt, altitude.getUom());
	}
	
	public static List<? extends AbstractSurfaceShape> getSurfaceShapes(SurfaceType surface) {
		List<AbstractSurfaceShape> surfaceShapes = new ArrayList<AbstractSurfaceShape>();
		
		List<JAXBElement<? extends AbstractSurfacePatchType>> patches =
				surface.getPatches().getValue().getAbstractSurfacePatch();
		for (JAXBElement<? extends AbstractSurfacePatchType> patch : patches) {
			surfaceShapes.add(GmlData.getSurfaceShape(patch.getValue()));
		}
		
		return surfaceShapes;
	}
	
	public static String getAirspaceType(AirspaceTimeSliceType airspaceTimeSlice) {
		return airspaceTimeSlice.getType().getValue().getValue();
	}
	
	public static String getAirspaceDesignator(AirspaceTimeSliceType airspaceTimeSlice) {
		return airspaceTimeSlice.getDesignator().getValue().getValue();
	}
	
	public static String getAirspaceName(AirspaceTimeSliceType airspaceTimeSlice) {		
		return airspaceTimeSlice.getAirspaceName().getValue().getValue();
	}
	
}
