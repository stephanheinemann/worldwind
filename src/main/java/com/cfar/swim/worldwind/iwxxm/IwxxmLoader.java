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

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;

import org.xml.sax.InputSource;

import com.cfar.swim.iwxxm.bind.IwxxmUnmarshaller;
import com.cfar.swim.worldwind.data.IwxxmData;
import com.cfar.swim.worldwind.data.ObstacleLoader;
import com.cfar.swim.worldwind.data.OmData;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostMap;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstaclePath;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;
import com.cfar.swim.worldwind.render.annotations.DepictionAnnotation;
import com.cfar.swim.worldwind.util.Depiction;
import com.cfar.swim.worldwind.weather.WeatherSymbolMap;

import gov.nasa.worldwind.render.airspaces.Airspace;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525GraphicFactory;
import icao.iwxxm.SIGMETReportStatusType;
import icao.iwxxm.SIGMETType;
import net.opengis.om.OMObservationType;

public class IwxxmLoader implements ObstacleLoader {

	private IwxxmUnmarshaller unmarshaller;
	
	private CostMap costMap;
	private WeatherSymbolMap wxSymbolMap;
	private MilStd2525GraphicFactory symbolFactory;
	
	public IwxxmLoader() throws JAXBException {
		this.unmarshaller = new IwxxmUnmarshaller();
		this.costMap = new CostMap();
		this.wxSymbolMap = new WeatherSymbolMap();
		this.symbolFactory = new MilStd2525GraphicFactory();
	}
	
	@Override
	public Set<Obstacle> load(InputSource source) throws JAXBException {
		Set<Obstacle> obstacles = null;
		JAXBElement<?> iwxxmElement = (JAXBElement<?>) this.unmarshaller.unmarshal(source);
		
		if (iwxxmElement.getValue() instanceof SIGMETType) {
			obstacles = this.loadSigmet((SIGMETType) iwxxmElement.getValue());
		}
		
		return obstacles;
	}
	
	public Set<Obstacle> loadSigmet(SIGMETType sigmet) throws JAXBException {
		if (sigmet.getStatus().equals(SIGMETReportStatusType.CANCELLATION)) {
			// TODO: create obstacles with negative costs
			// TODO: scenario should remove obstacles with negative costs
			return null;// this.loadCancelSigmet(sigmet);
		} else if (sigmet.getStatus().equals(SIGMETReportStatusType.NORMAL)) {
			return this.loadNormalSigmet(sigmet);
		}
		return null;
	}
	
	private Set<Obstacle> loadNormalSigmet(SIGMETType sigmet) throws JAXBException {
		Set<Obstacle> obstacles = new HashSet<Obstacle>();
		
		String phenomenom = IwxxmData.getPhenomenon(sigmet);
		int cost = 0;
		String sidc = null;
		
		if (null != phenomenom) {
			cost = this.costMap.get(phenomenom);
			sidc = this.wxSymbolMap.get(phenomenom);
		}
			
		List<OMObservationType> observations = IwxxmData.getObservations(sigmet);
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
			List<Airspace> observationAirspaces = OmData.getAirspaces(observation, this.unmarshaller);
			
			for (Airspace airspace : observationAirspaces) {
				if (airspace instanceof ObstacleCylinder) {
					ObstacleCylinder obstacle = (ObstacleCylinder) airspace;
					obstacle.setCostInterval(costInterval);
					
					obstacle.setDepiction(new Depiction(
							symbolFactory.createPoint(sidc, obstacle.getReferencePosition(), null)));
					obstacle.getDepiction().setAnnotation(new DepictionAnnotation(costInterval.getId(), obstacle.getReferencePosition()));
				}
			}
			
			sigmetAirspaces.addAll(observationAirspaces);
		}
		
		// cancel messages rely on SIGMET references instead of SIGMET identifiers
		//IwxxmSigmetReference sigmetReference = IwxxmData.getSigmetReference(sigmet);
		
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
			
				obstacles.add((ObstacleCylinder) current);
				obstacles.addAll(interpolants);
				
				//this.addSigmetObstacle(sigmetReference, (ObstacleCylinder) current);
				
				for (ObstacleCylinder interpolant : interpolants) {
					//this.addSigmetObstacle(sigmetReference, interpolant);
					interpolant.setDepiction(
							new Depiction(symbolFactory.createPoint(sidc, interpolant.getReferencePosition(), null)));
					interpolant.getDepiction().setAnnotation(
							new DepictionAnnotation(interpolant.getCostInterval().getId(), interpolant.getReferencePosition()));
				}
			}
			current = next;
		}
		
		if (null != current) {
			obstacles.add((ObstacleCylinder) current);
			//this.addSigmetObstacle(sigmetReference, (ObstacleCylinder) current);
			//this.idReferences.put(sigmet.getId(), sigmetReference);
		}
		
		// create observation / obstacle path
		ObstaclePath observationPath = (ObstaclePath) OmData.getAirspacePath(sigmetAirspaces);
		CostInterval pathCostInverval = new CostInterval(sigmet.getId(), IwxxmData.getValidPeriod(sigmet), cost);
		observationPath.setCostInterval(pathCostInverval);
		//sigmetObstacles.get(sigmetReference).add(observationPath);
		
		if (1 < sigmetAirspaces.size()) {
			//TODO: obstacle path does not have extent 
			//obstacles.add(observationPath);
		}
		
		return obstacles;
	}

}
