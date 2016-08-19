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
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;
import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamReader;

import org.xml.sax.InputSource;

import com.cfar.swim.iwxxm.bind.IwxxmUnmarshaller;
import com.cfar.swim.worldwind.data.IwxxmData;
import com.cfar.swim.worldwind.data.OmData;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostMap;
import com.cfar.swim.worldwind.planning.NonUniformCostIntervalGrid;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.render.CostIntervalCylinder;
import com.cfar.swim.worldwind.render.CostIntervalPath;

import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.render.RigidShape;
import gov.nasa.worldwind.util.measure.LengthMeasurer;
import icao.iwxxm.SIGMETType;
import net.opengis.gml.CircleByCenterPointType;
import net.opengis.om.OMObservationType;

public class IwxxmUpdater implements Runnable {
	
	CostMap costMap = new CostMap();
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
		List<Layer> renderableLayers = model.getLayers().getLayersByClass(RenderableLayer.class);
		RenderableLayer renderableLayer = ((RenderableLayer) renderableLayers.get(0));
		
		String phenomenom = IwxxmData.getPhenomenon(sigmet);
		// TODO: things like quantification (weakening), spacial and temporal distance
		// observation and analysis methods could be taken into account
		int cost = this.costMap.get(phenomenom);
		
		List<OMObservationType> observations = IwxxmData.getObservations(sigmet);
		List<RigidShape> observationShapes = new ArrayList<RigidShape>();
		//List<RigidShape> sigmetShapes = IwxxmData.getRigidShapes(sigmet, iwxxmUnmarshaller);
		
		for (OMObservationType observation : observations) {
			// phenomenonTime is now if not specified
			ZonedDateTime phenomenonTime = OmData.getPhenomenonTime(observation);
			ZonedDateTime resultTime = OmData.getResultTime(observation);
			//TimeInterval validInterval = OmData.getValidTimeInterval(observation);
			
			// only future times are actually relevant
			// phenomenon time can be a forecasted time
			// should the interval be the same as the time step interval?
			
			// if there is no forecast or observation time, take the time the data has been made available  
			ZonedDateTime start = phenomenonTime;
			if (null == start) {
				start = resultTime;
			}
			
			ZonedDateTime end = null;
			// the end time should be the valid time of the observation
			// if interpolation is done, then the final valid times have to be changed accordingly
			TimeInterval validTimeInterval = OmData.getValidTimeInterval(observation);
			if (null == validTimeInterval) {
				end = IwxxmData.getValidPeriod(sigmet).getUpper();
			} else {
				end = validTimeInterval.getUpper();
			}
			
			CostInterval costInterval = new CostInterval(sigmet.getId(), start, end.plusMinutes(30));
			costInterval.setCost(cost);
			System.out.println("cost interval = " + start + " ... " + end);
			System.out.println("cost = " + cost);
			System.out.println("time = " + ZonedDateTime.now(ZoneId.of("UTC")));
			
			List<RigidShape> sigmetShapes = OmData.getRigidShapes(observation, iwxxmUnmarshaller);
			observationShapes.addAll(sigmetShapes);
			//System.out.println("found shapes = " + sigmetShapes.size());
			//renderableLayer.addRenderables(sigmetShapes);
		
			// cylinder embedding test
			
			for (RigidShape shape : sigmetShapes) {
				if (shape instanceof CostIntervalCylinder) {
					CostIntervalCylinder costIntervalCylinder = (CostIntervalCylinder) shape;
					costIntervalCylinder.setCostInterval(costInterval);
					
					/*
					Cylinder cylinder = costIntervalCylinder.toGeometricCylinder(model.getGlobe());
					
					renderableLayer.addRenderable(costIntervalCylinder);
					//renderableLayer.removeRenderable(shape);
					//renderableLayer.addRenderable(cylinder);
					
					this.grid.embed(cylinder, costInterval); // identify affected cells
					*/
				}
			}
			
		}
		
		Iterator<RigidShape> osi = observationShapes.iterator();
		RigidShape current = null;
		if (osi.hasNext()) {
			current = osi.next();
		}
		while (osi.hasNext()) {
			RigidShape next = osi.next();
			if ((current instanceof CostIntervalCylinder) && (next instanceof CostIntervalCylinder)) {
				List<CostIntervalCylinder> interpolants = ((CostIntervalCylinder) current).interpolate((CostIntervalCylinder) next, 4);
			
				renderableLayer.addRenderable(current);
				renderableLayer.addRenderables(interpolants);
				
				Cylinder cylinder = ((CostIntervalCylinder) current).toGeometricCylinder(model.getGlobe());
				this.grid.embed(cylinder, ((CostIntervalCylinder) current).getCostInterval());
			
				for (CostIntervalCylinder interpolant : interpolants) {
					cylinder = interpolant.toGeometricCylinder(model.getGlobe());
					this.grid.embed(cylinder, interpolant.getCostInterval());
				}
			}
			current = next;
		}
		renderableLayer.addRenderable(current);
		Cylinder cylinder = ((CostIntervalCylinder) current).toGeometricCylinder(model.getGlobe());
		this.grid.embed(cylinder, ((CostIntervalCylinder) current).getCostInterval());
		
		CostIntervalPath observationPath = (CostIntervalPath) OmData.getObservationPath(observationShapes);
		// TODO: use valid times for path
		CostInterval pathCostInverval = new CostInterval(sigmet.getId(), IwxxmData.getValidPeriod(sigmet), cost);
		observationPath.setCostInterval(pathCostInverval);
		if (1 < observationShapes.size()) {
			renderableLayer.addRenderable(observationPath);
		}
		
		ArrayList<Position> positions = new ArrayList<Position>();
		for (Position position : observationPath.getPositions()) {
			positions.add(position);
		}
		LengthMeasurer measurer = new LengthMeasurer(positions);
		double meters = measurer.getLength(model.getGlobe());
		System.out.println("observation path length = " + meters);
	}
	
	/*
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
	*/
}
