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
import com.cfar.swim.worldwind.data.GmlData;
import com.cfar.swim.worldwind.data.IwxxmData;
import com.cfar.swim.worldwind.data.OmData;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.NonUniformCostIntervalGrid;
import com.cfar.swim.worldwind.planning.TimeInterval;

import aero.aixm.AirspaceVolumeType;
import aero.aixm.SurfaceType;
import aero.aixm.ValDistanceVerticalType;
import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.AbstractSurfaceShape;
import gov.nasa.worldwind.render.Cylinder;
import gov.nasa.worldwind.render.Renderable;
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
	Model model = null;
	NonUniformCostIntervalGrid grid = null;
	
	public IwxxmUpdater(/*InputSource source,*/ Model model, NonUniformCostIntervalGrid grid) throws JAXBException {
		this.iwxxmUnmarshaller = new IwxxmUnmarshaller();
		//this.source = source;
		this.model = model;
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
	
	public void processSigmet(SIGMETType sigmet) throws JAXBException {
		//this.isRelevantTime(sigmet);
		
		//sigmet.getPhenomenon(); // e.g. find thunderstorm
		// result/EvolvingMeteorologicalCondition/geometry
		// result/MeteorologicalPositionCollection/member/MeteorologicalPosition/geometry
		
		List<RigidShape> sigmetShapes = IwxxmData.getRigidShapes(sigmet, iwxxmUnmarshaller);
		System.out.println("found shapes = " + sigmetShapes.size());
		List<Layer> renderableLayers = model.getLayers().getLayersByClass(RenderableLayer.class);
		((RenderableLayer) renderableLayers.get(0)).addRenderables(sigmetShapes);
	}
	
	public void update(Cylinder cylinder) {
		// TODO: cost, time interval..., direction, speed
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
				Vec4 modelPoint = model.getGlobe().computePointFromLocation(location);
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
	
	
	
}
