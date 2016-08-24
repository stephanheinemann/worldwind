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
import com.cfar.swim.worldwind.render.ObstacleCylinder;
import com.cfar.swim.worldwind.render.ObstaclePath;

import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.RigidShape;
import icao.iwxxm.SIGMETReportStatusType;
import icao.iwxxm.SIGMETType;
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
		if (iwxxmElement.getValue() instanceof SIGMETType) {
			this.processSigmet((SIGMETType) iwxxmElement.getValue());
		}
	}

	public void process(InputStream iwxxmStream) throws XMLStreamException {
		this.xsr = this.xif.createXMLStreamReader(iwxxmStream);
		this.xsr = this.xif.createFilteredReader(this.xsr, new IwxxmPositionFilter());
	}
	
	public void processSigmet(SIGMETType sigmet) throws JAXBException {
		// TODO: consider sequence numbers
		if (sigmet.getStatus().equals(SIGMETReportStatusType.CANCELLATION)) {
			cancelSigmet(sigmet);
		} else if (sigmet.getStatus().equals(SIGMETReportStatusType.NORMAL)) {
			updateSigmet(sigmet);
		}
	}
	
	public void cancelSigmet(SIGMETType sigmet) {
		
	}
	
	public void updateSigmet(SIGMETType sigmet) throws JAXBException {
		List<Layer> renderableLayers = model.getLayers().getLayersByClass(RenderableLayer.class);
		RenderableLayer renderableLayer = ((RenderableLayer) renderableLayers.get(0));
		
		String phenomenom = IwxxmData.getPhenomenon(sigmet);
		// TODO: things like quantification (weakening), spatial and temporal distance
		// observation and analysis methods could be taken into account
		int cost = 0;
		if (null != phenomenom) {
			cost = this.costMap.get(phenomenom);
		}
			
		List<OMObservationType> observations = IwxxmData.getObservations(sigmet);
		List<RigidShape> sigmetShapes = new ArrayList<RigidShape>();
		
		for (OMObservationType observation : observations) {
			ZonedDateTime phenomenonTime = OmData.getPhenomenonTime(observation);
			ZonedDateTime resultTime = OmData.getResultTime(observation);
			TimeInterval validTimeInterval = OmData.getValidTimeInterval(observation);
			
			// if there is no forecast or observation time, use the time the
			// data has been made available  
			ZonedDateTime start = phenomenonTime;
			if (null == start) {
				start = resultTime;
			}
			
			// the end time should be the valid time of the observation or
			// otherwise the valid time of the sigmet
			// if interpolation is done, then the final valid times have to be
			// changed accordingly
			ZonedDateTime end = null;
			if (null == validTimeInterval) {
				end = IwxxmData.getValidPeriod(sigmet).getUpper();
			} else {
				end = validTimeInterval.getUpper();
			}
			
			CostInterval costInterval = new CostInterval(sigmet.getId(), start, end, cost);
			List<RigidShape> observationShapes = OmData.getRigidShapes(observation, iwxxmUnmarshaller);
			
			for (RigidShape shape : observationShapes) {
				if (shape instanceof ObstacleCylinder) {
					ObstacleCylinder obstacleCylinder = (ObstacleCylinder) shape;
					obstacleCylinder.setCostInterval(costInterval);
				}
			}
			
			sigmetShapes.addAll(observationShapes);
		}
		
		// TODO: assuming all shapes are ordered and belong to the same phenomenon
		// TODO: ensure ordering and related phenomenon
		Iterator<RigidShape> ssi = sigmetShapes.iterator();
		RigidShape current = null;
		
		if (ssi.hasNext()) {
			current = ssi.next();
		}
		
		while (ssi.hasNext()) {
			RigidShape next = ssi.next();
			if ((current instanceof ObstacleCylinder) && (next instanceof ObstacleCylinder)) {
				List<ObstacleCylinder> interpolants = ((ObstacleCylinder) current).interpolate((ObstacleCylinder) next, 4);
			
				renderableLayer.addRenderable(current);
				renderableLayer.addRenderables(interpolants);
				
				Cylinder cylinder = ((ObstacleCylinder) current).toGeometricCylinder(model.getGlobe());
				this.grid.embed(cylinder, ((ObstacleCylinder) current).getCostInterval());
			
				for (ObstacleCylinder interpolant : interpolants) {
					cylinder = interpolant.toGeometricCylinder(model.getGlobe());
					this.grid.embed(cylinder, interpolant.getCostInterval());
				}
			}
			current = next;
		}
		
		if (null != current) {
			renderableLayer.addRenderable(current);
			Cylinder cylinder = ((ObstacleCylinder) current).toGeometricCylinder(model.getGlobe());
			this.grid.embed(cylinder, ((ObstacleCylinder) current).getCostInterval());
		}
		
		// TODO: use valid times for path
		ObstaclePath observationPath = (ObstaclePath) OmData.getObservationPath(sigmetShapes);
		CostInterval pathCostInverval = new CostInterval(sigmet.getId(), IwxxmData.getValidPeriod(sigmet), cost);
		observationPath.setCostInterval(pathCostInverval);
		
		if (1 < sigmetShapes.size()) {
			renderableLayer.addRenderable(observationPath);
		}
		
		/*
		ArrayList<Position> positions = new ArrayList<Position>();
		for (Position position : observationPath.getPositions()) {
			positions.add(position);
		}
		LengthMeasurer measurer = new LengthMeasurer(positions);
		double meters = measurer.getLength(model.getGlobe());
		System.out.println("observation path length = " + meters);
		*/
	}
	
}
