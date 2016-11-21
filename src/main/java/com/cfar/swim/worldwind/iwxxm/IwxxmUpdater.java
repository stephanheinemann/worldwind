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
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamReader;

import org.xml.sax.InputSource;

import com.cfar.swim.iwxxm.bind.IwxxmUnmarshaller;
import com.cfar.swim.worldwind.data.DataActivationListener;
import com.cfar.swim.worldwind.data.IwxxmData;
import com.cfar.swim.worldwind.data.OmData;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostMap;
import com.cfar.swim.worldwind.planning.PlanningGrid;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstaclePath;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;
import com.cfar.swim.worldwind.util.Depiction;
import com.cfar.swim.worldwind.weather.WeatherSymbolMap;

import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.GlobeAnnotation;
import gov.nasa.worldwind.render.airspaces.Airspace;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525GraphicFactory;
import icao.iwxxm.SIGMETReportStatusType;
import icao.iwxxm.SIGMETType;
import net.opengis.om.OMObservationType;

public class IwxxmUpdater implements DataActivationListener, Runnable {
	
	CostMap costMap = new CostMap();
	WeatherSymbolMap wxSymbolMap = new WeatherSymbolMap();
	MilStd2525GraphicFactory symbolFactory = new MilStd2525GraphicFactory();
	
	XMLInputFactory xif = XMLInputFactory.newFactory();
	XMLStreamReader xsr;
	
	IwxxmUnmarshaller iwxxmUnmarshaller = null;
	Unmarshaller unmarshaller = null;
	//InputSource source = null;
	Model model = null;
	
	PlanningGrid grid = null;
	RenderableLayer renderableLayer = null;
	
	/** the SIGMET embeddings this updater has created (references are required for canceling messages) */
	HashMap<IwxxmSigmetReference, List<Obstacle>> sigmetObstacles = new HashMap<IwxxmSigmetReference, List<Obstacle>>();
	
	/** the SIGMET identifier to reference mapping */
	HashMap<String, IwxxmSigmetReference> idReferences = new HashMap<String, IwxxmSigmetReference>();
	
	public IwxxmUpdater(/*InputSource source,*/ Model model, PlanningGrid grid) throws JAXBException {
		this.iwxxmUnmarshaller = new IwxxmUnmarshaller();
		// TODO: repair IwxxmUnmarshaller and others to include ALL object factories!
		this.unmarshaller = JAXBContext.newInstance(
				icao.iwxxm.ObjectFactory.class,
				net.opengis.sampling.ObjectFactory.class).createUnmarshaller();
		//this.source = source;
		this.grid = grid;
		List<Layer> renderableLayers = model.getLayers().getLayersByClass(RenderableLayer.class);
		this.renderableLayer = ((RenderableLayer) renderableLayers.get(0));
	}
	
	@Override
	public void run() {
		// TODO: continuously check existing data for validity
		// TODO: insert new data while possibly increasing the resolution
		// TODO: remove obsolete data while possible decreasing the resolution
		// TODO: should monitor time (maybe a more general runnable using updaters?)
	}
	
	public void add(InputSource source) throws JAXBException {
		JAXBElement<?> iwxxmElement = (JAXBElement<?>) this.unmarshaller.unmarshal(source);
		
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
		try {
			if (sigmet.getStatus().equals(SIGMETReportStatusType.CANCELLATION)) {
				this.cancelSigmet(sigmet);
			} else if (sigmet.getStatus().equals(SIGMETReportStatusType.NORMAL)) {
				this.updateSigmet(sigmet);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	public void cancelSigmet(SIGMETType sigmet) throws JAXBException {
		int cancelledSequenceNumber = IwxxmData.getCancelledSequenceNumber(sigmet);
		TimeInterval cancelledValidPeriod = IwxxmData.getCancelledValidPeriod(sigmet);
		IwxxmSigmetReference sigmetReference = IwxxmData.getSigmetReference(sigmet);
		// cancel from the start of the cancel validity period until the end of the validity period of the canceled message
		TimeInterval cancelInterval = new TimeInterval(sigmetReference.getValidStart(), cancelledValidPeriod.getUpper());
		
		// build SIGMET reference
		IwxxmSigmetReference cancelReference = new IwxxmSigmetReference(
				sigmetReference.getAirspaceDesignator(),
				sigmetReference.getAirspaceName(),
				cancelledSequenceNumber,
				cancelledValidPeriod.getLower(),
				cancelledValidPeriod.getUpper());
		
		// find obstacles of canceled SIGMET
		List<Obstacle> obstacles = this.sigmetObstacles.get(cancelReference);
		for (Obstacle obstacle : obstacles) {
			CostInterval costInterval = obstacle.getCostInterval();
			
			if (costInterval.intersects(cancelInterval)) {
				// cancel intersecting interval with negative cost obstacle
				CostInterval intersectionInterval = new CostInterval(
						sigmet.getId(),
						costInterval.intersect(cancelInterval),
						-costInterval.getCost());
				
				// embed canceling obstacle
				if (obstacle instanceof ObstacleCylinder) {
					ObstacleCylinder obstacleCylinder = (ObstacleCylinder) obstacle;
					ObstacleCylinder intersectionObstacle = new ObstacleCylinder(
							obstacleCylinder.getCenter(),
							obstacleCylinder.getAltitudes()[0],
							obstacleCylinder.getAltitudes()[1],
							obstacleCylinder.getRadii()[1]);
					intersectionObstacle.setCostInterval(intersectionInterval);
					if (intersectionObstacle.hasDepiction()) {
						if (intersectionObstacle.getDepiction().hasAnnotation()) {
							intersectionObstacle.getDepiction().getAnnotation().setText(intersectionInterval.getId());
						} else {
							intersectionObstacle.getDepiction().setAnnotation(
								new GlobeAnnotation(intersectionInterval.getId(), intersectionObstacle.getReferencePosition()));
						}
					}
					
					this.grid.embed(intersectionObstacle);
					this.addSigmetObstacle(sigmetReference, intersectionObstacle);
					this.idReferences.put(sigmet.getId(), sigmetReference);
					this.renderableLayer.addRenderable(intersectionObstacle);
				}
				// TODO: implement other obstacle types
			}
		}
	}
	
	public void updateSigmet(SIGMETType sigmet) throws JAXBException {
		String phenomenom = IwxxmData.getPhenomenon(sigmet);
		// TODO: things like quantification (weakening), spatial and temporal distance
		// observation and analysis methods could be taken into account
		int cost = 0;
		String sidc = null;
		
		if (null != phenomenom) {
			cost = this.costMap.get(phenomenom);
			sidc = this.wxSymbolMap.get(phenomenom);
		}
			
		List<OMObservationType> observations = IwxxmData.getObservations(sigmet);
		System.out.println("found " + observations.size() + " observations for " + sigmet.getId());
		List<Airspace> sigmetAirspaces = new ArrayList<Airspace>();
		
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
			// otherwise the valid time of the SIGMET
			// if interpolation is done, then the final valid times have to be
			// changed accordingly
			ZonedDateTime end = null;
			if (null == validTimeInterval) {
				end = IwxxmData.getValidPeriod(sigmet).getUpper();
			} else {
				end = validTimeInterval.getUpper();
			}
			
			CostInterval costInterval = new CostInterval(sigmet.getId(), start, end, cost);
			List<Airspace> observationAirspaces = OmData.getAirspaces(observation, iwxxmUnmarshaller);
			
			for (Airspace airspace : observationAirspaces) {
				if (airspace instanceof ObstacleCylinder) {
					ObstacleCylinder obstacle = (ObstacleCylinder) airspace;
					obstacle.setCostInterval(costInterval);
					
					obstacle.setDepiction(new Depiction(
							symbolFactory.createPoint(sidc, obstacle.getReferencePosition(), null)));
					obstacle.getDepiction().setAnnotation(new GlobeAnnotation(costInterval.getId(), obstacle.getReferencePosition()));
				}
			}
			
			sigmetAirspaces.addAll(observationAirspaces);
		}
		
		// cancel messages rely on SIGMET references instead of SIGMET identifiers
		IwxxmSigmetReference sigmetReference = IwxxmData.getSigmetReference(sigmet);
		
		// TODO: assuming all shapes are ordered and belong to the same phenomenon
		// TODO: ensure ordering and related phenomenon
		Iterator<Airspace> ssi = sigmetAirspaces.iterator();
		Airspace current = null;
		
		if (ssi.hasNext()) {
			current = ssi.next();
		}
		
		while (ssi.hasNext()) {
			Airspace next = ssi.next();
			if ((current instanceof ObstacleCylinder) && (next instanceof ObstacleCylinder)) {
				List<ObstacleCylinder> interpolants = ((ObstacleCylinder) current).interpolate((ObstacleCylinder) next, 4);
			
				this.renderableLayer.addRenderable(current);
				this.renderableLayer.addRenderables(interpolants);
				this.grid.embed((ObstacleCylinder) current);
				this.addSigmetObstacle(sigmetReference, (ObstacleCylinder) current);
				
				for (ObstacleCylinder interpolant : interpolants) {
					this.grid.embed(interpolant);
					this.addSigmetObstacle(sigmetReference, interpolant);
					
					interpolant.setDepiction(new Depiction(
							symbolFactory.createPoint(sidc, interpolant.getReferencePosition(), null)));
				}
			}
			current = next;
		}
		
		if (null != current) {
			this.renderableLayer.addRenderable(current);
			this.grid.embed((ObstacleCylinder) current);
			this.addSigmetObstacle(sigmetReference, (ObstacleCylinder) current);
			this.idReferences.put(sigmet.getId(), sigmetReference);
		}
		
		// create observation / obstacle path
		ObstaclePath observationPath = (ObstaclePath) OmData.getAirspacePath(sigmetAirspaces);
		CostInterval pathCostInverval = new CostInterval(sigmet.getId(), IwxxmData.getValidPeriod(sigmet), cost);
		observationPath.setCostInterval(pathCostInverval);
		sigmetObstacles.get(sigmetReference).add(observationPath);
		
		if (1 < sigmetAirspaces.size()) {
			this.renderableLayer.addRenderable(observationPath);
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
	
	private void addSigmetObstacle(IwxxmSigmetReference sigmetReference, Obstacle obstacle) {
		if (this.sigmetObstacles.containsKey(sigmetReference)) {
			this.sigmetObstacles.get(sigmetReference).add(obstacle);
		} else {
			ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>();
			obstacles.add(obstacle);
			this.sigmetObstacles.put(sigmetReference, obstacles);
		}
	}

	@Override
	public void activate(String identifier) {
		if (this.idReferences.containsKey(identifier)) {
			IwxxmSigmetReference sigmetReference = (IwxxmSigmetReference) this.idReferences.get(identifier);
			List<Obstacle> obstacles = this.sigmetObstacles.get(sigmetReference);
			for (Obstacle obstacle : obstacles) {
				obstacle.enable();
				this.grid.refresh(obstacle);
			}
		}
	}

	@Override
	public void deactivate(String identifier) {
		if (this.idReferences.containsKey(identifier)) {
			IwxxmSigmetReference sigmetReference = (IwxxmSigmetReference) this.idReferences.get(identifier);
			List<Obstacle> obstacles = this.sigmetObstacles.get(sigmetReference);
			for (Obstacle obstacle : obstacles) {
				obstacle.disable();
				this.grid.refresh(obstacle);
			}
		}
	}

	@Override
	public Set<String> getIdentifiers() {
		return this.idReferences.keySet();
	}
	
}
