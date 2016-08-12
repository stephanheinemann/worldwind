package com.cfar.swim.worldwind.data;

import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBElement;

import com.cfar.swim.worldwind.render.VerticalCylinder;

import aero.aixm.AirspaceVolumeType;
import aero.aixm.SurfaceType;
import aero.aixm.ValDistanceVerticalType;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.AbstractSurfaceShape;
import gov.nasa.worldwind.render.RigidShape;
import gov.nasa.worldwind.render.SurfaceCircle;
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
					rigidShapes.add(new VerticalCylinder(center, height, radius));
				}
			}
		}	
		
		return rigidShapes;
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
	
	
}
