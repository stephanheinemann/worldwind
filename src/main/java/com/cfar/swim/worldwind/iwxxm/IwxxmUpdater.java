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
package com.cfar.swim.worldwind.iwxxm;

import java.io.InputStream;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;
import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamReader;

import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;

import com.cfar.swim.iwxxm.bind.IwxxmUnmarshaller;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.NonUniformCostIntervalGrid;
import com.cfar.swim.worldwind.planning.TimeInterval;

import aero.aixm.AirspaceVolumeType;
import aero.aixm.SurfaceType;
import aero.aixm.ValDistanceVerticalType;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.AbstractSurfaceShape;
import gov.nasa.worldwind.render.Cylinder;
import gov.nasa.worldwind.render.RigidShape;
import gov.nasa.worldwind.render.SurfaceCircle;
import gov.nasa.worldwind.render.SurfacePolygon;
import icao.iwxxm.EvolvingMeteorologicalConditionType;
import icao.iwxxm.MeteorologicalPositionCollectionPropertyType;
import icao.iwxxm.MeteorologicalPositionCollectionType;
import icao.iwxxm.MeteorologicalPositionPropertyType;
import icao.iwxxm.MeteorologicalPositionType;
import icao.iwxxm.ObjectFactory;
import icao.iwxxm.SIGMETType;
import icao.iwxxm.TAFType;
import net.opengis.gml.AbstractCurveSegmentType;
import net.opengis.gml.AbstractCurveType;
import net.opengis.gml.AbstractRingType;
import net.opengis.gml.AbstractSurfacePatchType;
import net.opengis.gml.ArcByBulgeType;
import net.opengis.gml.ArcByCenterPointType;
import net.opengis.gml.ArcStringByBulgeType;
import net.opengis.gml.ArcStringType;
import net.opengis.gml.ArcType;
import net.opengis.gml.BSplineType;
import net.opengis.gml.BezierType;
import net.opengis.gml.CircleByCenterPointType;
import net.opengis.gml.CircleType;
import net.opengis.gml.ClothoidType;
import net.opengis.gml.CompositeCurveType;
import net.opengis.gml.ConeType;
import net.opengis.gml.CubicSplineType;
import net.opengis.gml.CurvePropertyType;
import net.opengis.gml.CurveType;
import net.opengis.gml.CylinderType;
import net.opengis.gml.GeodesicStringType;
import net.opengis.gml.GeodesicType;
import net.opengis.gml.LineStringSegmentType;
import net.opengis.gml.LineStringType;
import net.opengis.gml.LinearRingType;
import net.opengis.gml.OffsetCurveType;
import net.opengis.gml.OrientableCurveType;
import net.opengis.gml.PolygonPatchType;
import net.opengis.gml.RectangleType;
import net.opengis.gml.RingType;
import net.opengis.gml.SphereType;
import net.opengis.gml.TimeInstantType;
import net.opengis.gml.TimePeriodType;
import net.opengis.gml.TimePositionType;
import net.opengis.gml.TriangleType;
import net.opengis.om.OMObservationPropertyType;

public class IwxxmUpdater implements Runnable {

	XMLInputFactory xif = XMLInputFactory.newFactory();
	XMLStreamReader xsr;
	
	IwxxmUnmarshaller iwxxmUnmarshaller = null;
	//InputSource source = null;
	Globe globe = null;
	NonUniformCostIntervalGrid grid = null;
	
	public IwxxmUpdater(/*InputSource source,*/ Globe globe, NonUniformCostIntervalGrid grid) throws JAXBException {
		this.iwxxmUnmarshaller = new IwxxmUnmarshaller();
		//this.source = source;
		this.globe = globe;
		this.grid = grid;
	}
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		
		// TODO: continuously check existing data for validity
		// TODO: insert new data while possibly increasing the resolution
		// TODO: remove obsolete data while possible decreasing the resolution
		// TODO: should monitor time (maybe a more general runnable using updaters?)
	}
	
	public void add(InputSource source) throws JAXBException {
		JAXBElement<?> iwxxmElement = (JAXBElement<?>) this.iwxxmUnmarshaller.unmarshal(source);
		
		// TODO: examine elements of the tree (position check)
		// TODO: a filtered reader could be constructed from an XMLStreamReader
		// TODO: only observations that are contained in the grid are relevant
		// TODO: both time and space could be filtered...
		// TODO: jaxb-filtered-parsing
		if (iwxxmElement.getDeclaredType().equals(SIGMETType.class)) {
			this.processSigmet((SIGMETType) iwxxmElement.getValue());
		}
	}
	
	public void remove() {}

	public void process(InputStream iwxxmStream) throws XMLStreamException {
		this.xsr = this.xif.createXMLStreamReader(iwxxmStream);
		this.xsr = this.xif.createFilteredReader(this.xsr, new IwxxmPositionFilter());
	}
	
	protected boolean isRelevant(SIGMETType sigment) {
		return true;
	}
	
	protected boolean isRelevantSpace(SIGMETType sigmet) {
		return true;
	}
	
	protected boolean isRelevantTime(SIGMETType sigmet) {
		// the relevant duration is related to the radius
		// of action at now
		ZonedDateTime now = ZonedDateTime.now(ZoneId.of("UTC"));
		System.out.println("UTC time now = " + now);
		return true;
	}
	
	
	public static ZonedDateTime getIssueTime(TAFType taf) {
		return null;
	}
	
	public static ZonedDateTime getValidTime(TAFType taf) {
		return null;
	}
	
	public static TimeInterval getValidPeriod(TAFType taf) {
		return getTimeInterval(taf.getValidTime().getTimePeriod());
	}
	
	public static TimeInterval getValidPeriod(SIGMETType sigmet) {
		return getTimeInterval(sigmet.getValidPeriod().getTimePeriod());
	}
	
	public static TimeInterval getCancelledValidPeriod(SIGMETType sigmet) {
		return getTimeInterval(sigmet.getCancelledValidPeriod().getTimePeriod());
	}
	
	public static TimeInterval getTimeInterval(TimePeriodType timePeriod) {
		TimeInterval timeInterval = null;
		ZonedDateTime start = null;
		ZonedDateTime end = null;
		
		if (null != timePeriod.getBeginPosition()) {
			start = getTime(timePeriod.getBeginPosition());
		} else if (null != timePeriod.getBegin()) {
			start = getTime(timePeriod.getBegin().getTimeInstant().getTimePosition());
		}
		
		if (null != timePeriod.getEndPosition()) {
			end = getTime(timePeriod.getEndPosition());
		} else if (null != timePeriod.getEnd()) {
			end = getTime(timePeriod.getEnd().getTimeInstant().getTimePosition());
		}
		
		if ((null != start) && (null != end)) {
			timeInterval = new TimeInterval(start, end);
		}
		
		return timeInterval;
	} 
	
	
	public static ZonedDateTime getTime(TimePositionType timePosition) {
		ZonedDateTime time = null;
		
		// TODO: check the rest of the list
		List<String> timeInstances = timePosition.getValue();
		if (!timeInstances.isEmpty()) {
			String timeInstance = timeInstances.get(0);
			// TODO: check other valid formats
			time = ZonedDateTime.parse(timeInstance, DateTimeFormatter.ISO_INSTANT);
		}
		
		return time;
	}
	
	public static List<? extends RigidShape> getRigidShapes(AirspaceVolumeType airspace) {
		List<RigidShape> rigidShapes = new ArrayList<RigidShape>();
		double lowerLimit = 0.0;
		double upperLimit = 0.0;
		
		// TODO: incorporate references
		if (null != airspace.getLowerLimit()) {
			lowerLimit = getHeight(airspace.getLowerLimit().getValue());
		}
		
		if (null != airspace.getUpperLimit()) {
			upperLimit = getHeight(airspace.getUpperLimit().getValue());
		}
			
		// TODO: implement other airspace volume elements
		List<? extends AbstractSurfaceShape> surfaceShapes =
				getSurfaceShapes(airspace.getHorizontalProjection().getValue().getSurface().getValue());
		for (AbstractSurfaceShape surfaceShape : surfaceShapes) {
			if (surfaceShape instanceof SurfaceCircle) {
				if (upperLimit > lowerLimit) {
					double height = upperLimit - lowerLimit;
					double altitude = lowerLimit + (height / 2.0);
					LatLon location = ((SurfaceCircle) surfaceShape).getCenter();
					double radius = ((SurfaceCircle) surfaceShape).getRadius();
					Position center = new Position(location, altitude);
					rigidShapes.add(new Cylinder(center, height, radius));
				}
			}
		}	
		
		return rigidShapes;
	}
	
	public static double getHeight(ValDistanceVerticalType altitude) {
		// TODO: conversion to meters!
		// TODO: might be flight levels, feet etc.
		return Double.parseDouble(altitude.getValue());
	}
	
	public static List<? extends AbstractSurfaceShape> getSurfaceShapes(SurfaceType surface) {
		List<AbstractSurfaceShape> surfaceShapes = new ArrayList<AbstractSurfaceShape>();
		
		List<JAXBElement<? extends AbstractSurfacePatchType>> patches =
				surface.getPatches().getValue().getAbstractSurfacePatch();
		for (JAXBElement<? extends AbstractSurfacePatchType> patch : patches) {
			surfaceShapes.add(getSurfaceShape(patch.getValue()));
		}
		
		return surfaceShapes;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(AbstractSurfacePatchType patch) {
		AbstractSurfaceShape surfaceShape = null;
		
		// TODO: implement other concrete types
		if (patch instanceof ConeType) {
			
		} else if (patch instanceof CylinderType) {
			
		} else if (patch instanceof SphereType) {
			
		} else if (patch instanceof PolygonPatchType) {
			surfaceShape = getSurfaceShape((PolygonPatchType) patch);
		} else if (patch instanceof RectangleType) {
			
		} else if (patch instanceof TriangleType) {
			
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(PolygonPatchType polygonPatch) {
		AbstractSurfaceShape surfaceShape = null;
		
		if (null != polygonPatch.getExterior()) {
			surfaceShape = getSurfaceShape(polygonPatch.getExterior().getAbstractRing().getValue());
		}
		
		if (null != polygonPatch.getInterior()) {
			// TODO: implement interior shapes if required
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(AbstractRingType ring) {
		AbstractSurfaceShape surfaceShape = null;
		
		if (ring instanceof LinearRingType) {
			// TODO: implement linear ring type if required
		} else if (ring instanceof RingType) {
			surfaceShape = getSurfaceShape((RingType) ring);
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(RingType ring) {
		AbstractSurfaceShape surfaceShape = null;
		List<AbstractSurfaceShape> curveShapes = new ArrayList<AbstractSurfaceShape>();
		
		List<CurvePropertyType> curves = ring.getCurveMember();
		for (CurvePropertyType curve : curves) {
			curveShapes.add(getSurfaceShape(curve.getAbstractCurve().getValue()));
		}
		
		for (AbstractSurfaceShape curveShape : curveShapes) {
			// TODO: assemble curves to shape (check instances)
			if (curveShape instanceof SurfaceCircle) {
				surfaceShape = curveShape;
				break;
			}
		}
			
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(AbstractCurveType curve) {
		AbstractSurfaceShape surfaceShape = null;
		
		// TODO: implement other concrete types
		if (curve instanceof CompositeCurveType) {
			
		} else if (curve instanceof CurveType) {
			
		} else if (curve instanceof LineStringType) {
			
		} else if (curve instanceof OrientableCurveType) {
			
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(CurveType curve) {
		AbstractSurfaceShape surfaceShape = null;
		
		for (JAXBElement<? extends AbstractCurveSegmentType> segment : curve.getSegments().getAbstractCurveSegment()) {
			surfaceShape = getSurfaceShape(segment.getValue());
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(AbstractCurveSegmentType segment) {
		AbstractSurfaceShape surfaceShape = null;
		
		// TODO: implement other concrete types
		if (segment instanceof ArcByBulgeType) {
		
		} else if (segment instanceof ArcByCenterPointType) {
			
		} else if (segment instanceof ArcStringByBulgeType) {
			
		} else if (segment instanceof ArcStringType) {
		
		} else if (segment instanceof ArcType) {
		
		} else if (segment instanceof BezierType) {	
		
		} else if (segment instanceof BSplineType) {
		
		} else if (segment instanceof CircleByCenterPointType) {
			surfaceShape = getSurfaceCircle((CircleByCenterPointType) segment);
		} else if (segment instanceof CircleType) {
		
		} else if (segment instanceof ClothoidType) {
			
		} else if (segment instanceof CubicSplineType) {
		
		} else if (segment instanceof GeodesicType) {
		
		} else if (segment instanceof GeodesicStringType) {
			
		} else if (segment instanceof LineStringSegmentType) {
			
		} else if (segment instanceof OffsetCurveType) {
			
		}
		
		return surfaceShape;
	}
	
	public static SurfaceCircle getSurfaceCircle(CircleByCenterPointType circle) {
		LatLon center = null;
		
		// TODO: implement other possible elements
		if (null != circle.getPos()) {
			List<Double> position = circle.getPos().getValue();
			center = getLocations(position).get(0);
		}
		
		// TODO: conversion to meters has to be done!
		double radius = circle.getRadius().getValue();
		
		return new SurfaceCircle(center, radius);
	}
	
	public static List<LatLon> getLocations(List<Double> angles) {
		List<LatLon> locations = new ArrayList<LatLon>();
		Angle latitude = null;
		Angle longitude = null;
		
		int index = 0;
		for (Double angle : angles) {
			if (0 == index % 2) {
				latitude = Angle.fromDegrees(angle);
			} else {
				longitude = Angle.fromDegrees(angle);
				locations.add(new LatLon(latitude, longitude));
			}
			index++;
		}
		
		return locations;
	}
	
	public void processSigmet(SIGMETType sigmet) throws JAXBException {
		this.isRelevantTime(sigmet);
		
		sigmet.getPhenomenon(); // e.g. find thunderstorm
		// result/EvolvingMeteorologicalCondition/geometry
		// result/MeteorologicalPositionCollection/member/MeteorologicalPosition/geometry
		
		for (OMObservationPropertyType analysis : sigmet.getAnalysis()) {
			
			Node node = (Node) analysis.getOMObservation().getResult();
			if (node.hasChildNodes()) {
				NodeList children = node.getChildNodes();
				int length = children.getLength();
				for (int i = 0; i < length; i++) {
					Node child = children.item(i);
					
					if (Node.ELEMENT_NODE == child.getNodeType()) {
						JAXBElement<?> content = (JAXBElement<?>) this.iwxxmUnmarshaller.unmarshal(child);
						if (content.getDeclaredType().equals(MeteorologicalPositionCollectionType.class)) {
							update((MeteorologicalPositionCollectionType) content.getValue());
						} else if (content.getDeclaredType().equals(MeteorologicalPositionType.class)) {
							update((MeteorologicalPositionType) content.getValue());
						} else if (content.getDeclaredType().equals(EvolvingMeteorologicalConditionType.class)) {
							update((EvolvingMeteorologicalConditionType) content.getValue());
						}
					}
				}
			}
		}
	}
	
	public void update(MeteorologicalPositionCollectionType mpc) {
		for (MeteorologicalPositionPropertyType member : mpc.getMember()) {
			update(member.getMeteorologicalPosition());
		}
	}
	
	public void update(MeteorologicalPositionType mp) {
		SurfaceType surface = mp.getGeometry().getAirspaceVolume().getHorizontalProjection().getValue().getSurface().getValue();
		for (JAXBElement<? extends AbstractSurfacePatchType> patch : surface.getPatches().getValue().getAbstractSurfacePatch()) {
			update(patch.getValue());
		}
	}
	
	public void update(AbstractSurfacePatchType asp) {
		if (asp instanceof PolygonPatchType) {
			PolygonPatchType pp = (PolygonPatchType) asp;
			update(pp);
		}
		// ...
	}
	
	public void update(PolygonPatchType pp) {
		AbstractRingType ar = pp.getExterior().getAbstractRing().getValue();
		if (ar instanceof RingType) {
			update((RingType) ar);
		}
		// ...
	}
	
	public void update(RingType r) {
		// ...
		for (CurvePropertyType cp : r.getCurveMember()) {
			AbstractCurveType c = cp.getAbstractCurve().getValue();
			if (c instanceof CurveType) {
				update((CurveType) c);
			}
			// ...
		}
	}
	
	public void update(CurveType c) {
		for (JAXBElement<? extends AbstractCurveSegmentType> s : c.getSegments().getAbstractCurveSegment()) {
			update(s.getValue());
		}
	}
	
	public void update(AbstractCurveSegmentType acs) {
		if (acs instanceof CircleByCenterPointType) {
			update((CircleByCenterPointType) acs);
		}
		// ...
	}
	
	public void update(CircleByCenterPointType cbcp) {
		// ...
		Angle latitude = Angle.ZERO;
		Angle longitude = Angle.ZERO;
		int index = 0;
		for (Double angle : cbcp.getPos().getValue()) {
			System.out.println("angle = " + angle);
			
			// TODO: subList...
			// TODO: spliterator...
			if (0 == (index % 2)) {
				latitude = Angle.fromDegrees(angle);
			} else {
				longitude = Angle.fromDegrees(angle);
				LatLon location = new LatLon(latitude, longitude);
				Vec4 modelPoint = globe.computePointFromLocation(location);
				if (grid.contains(modelPoint)) {
					CostInterval costInterval = new CostInterval(
							ZonedDateTime.now(ZoneId.of("UTC")),
							ZonedDateTime.of(3000, 1, 1, 0, 0, 0, 0, ZoneId.of("UTC")));
					costInterval.setCost(99);
					// parent should aggregate any child costs
					grid.addCostInterval(costInterval);
					grid.setTime(ZonedDateTime.now(ZoneId.of("UTC")));
					System.out.println("found matching cell...");
					for (NonUniformCostIntervalGrid child : grid.lookupCells(modelPoint)) {	
						child.addCostInterval(costInterval);
					}
				}
			}
			
			index = (index + 1) % 2;
		}
	}
	
	public void update(EvolvingMeteorologicalConditionType emc) {
		SurfaceType surface = emc.getGeometry().getAirspaceVolume().getHorizontalProjection().getValue().getSurface().getValue();
		for (JAXBElement<? extends AbstractSurfacePatchType> patch : surface.getPatches().getValue().getAbstractSurfacePatch()) {
			update(patch.getValue());
		}
	}
	
}
