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
package com.cfar.swim.worldwind.data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBException;

import com.cfar.swim.iwxxm.bind.IwxxmUnmarshaller;
import com.cfar.swim.worldwind.iwxxm.IwxxmSigmetReference;
import com.cfar.swim.worldwind.planning.TimeInterval;

import aero.aixm.AirspaceTimeSlicePropertyType;
import aero.aixm.AirspaceTimeSliceType;
import aero.aixm.AirspaceType;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.render.RigidShape;
import gov.nasa.worldwind.render.airspaces.Airspace;
import icao.iwxxm.EvolvingMeteorologicalConditionType;
import icao.iwxxm.MeteorologicalPositionCollectionType;
import icao.iwxxm.MeteorologicalPositionPropertyType;
import icao.iwxxm.MeteorologicalPositionType;
import icao.iwxxm.SIGMETType;
import icao.iwxxm.TAFType;
import icao.iwxxm.TropicalCycloneSIGMETType;
import net.opengis.gml.FeaturePropertyType;
import net.opengis.om.OMObservationPropertyType;
import net.opengis.om.OMObservationType;
import net.opengis.sampling.spatial.SFSpatialSamplingFeatureType;

public class IwxxmData {
	
	public static IwxxmSigmetReference getSigmetReference(SIGMETType sigmet) {
		IwxxmSigmetReference sigmetReference = null;
		
		List<OMObservationType> observations = IwxxmData.getObservations(sigmet);
		if (0 < observations.size()) {
			FeaturePropertyType feature = observations.get(0).getFeatureOfInterest();
			if (null != feature) {
				Object object = feature.getAbstractFeature().getValue();
				if (object instanceof SFSpatialSamplingFeatureType) {
					SFSpatialSamplingFeatureType spatialSamplingFeature = (SFSpatialSamplingFeatureType) object;
					List<FeaturePropertyType> sampledFeatures = spatialSamplingFeature.getSampledFeature();
					if (0 < sampledFeatures.size()) {
						object = sampledFeatures.get(0).getAbstractFeature().getValue();
						if (object instanceof AirspaceType) {
							AirspaceType airspace = (AirspaceType) object;
							List<AirspaceTimeSlicePropertyType> airspaceTimeSliceProperties = airspace.getTimeSlice(); 
							if (0 < airspaceTimeSliceProperties.size()) {
								AirspaceTimeSliceType airspaceTimeSlice = airspaceTimeSliceProperties.get(0).getAirspaceTimeSlice();
								sigmetReference = new IwxxmSigmetReference(
										AixmData.getAirspaceDesignator(airspaceTimeSlice),
										AixmData.getAirspaceName(airspaceTimeSlice),
										IwxxmData.getSequenceNumber(sigmet),
										IwxxmData.getValidPeriod(sigmet).getLower(),
										IwxxmData.getValidPeriod(sigmet).getUpper());
							}
						}
					}
				}
			}
		}
		
		return sigmetReference;
	}
	
	public static int getSequenceNumber(SIGMETType sigmet) {
		return Integer.parseInt(sigmet.getSequenceNumber());
	}
	
	public static int getCancelledSequenceNumber(SIGMETType sigmet) {
		return Integer.parseInt(sigmet.getCancelledSequenceNumber());
	}
	
	public static ZonedDateTime getIssueTime(TAFType taf) {
		return null;
	}
	
	public static ZonedDateTime getValidTime(TAFType taf) {
		return null;
	}
	
	public static TimeInterval getValidPeriod(TAFType taf) {
		return GmlData.getTimeInterval(taf.getValidTime().getTimePeriod());
	}
	
	public static TimeInterval getValidPeriod(SIGMETType sigmet) {
		return GmlData.getTimeInterval(sigmet.getValidPeriod().getTimePeriod());
	}
	
	public static TimeInterval getCancelledValidPeriod(SIGMETType sigmet) {
		return GmlData.getTimeInterval(sigmet.getCancelledValidPeriod().getTimePeriod());
	}
	
	public static String getPhenomenon(SIGMETType sigmet) {
		return sigmet.getPhenomenon().getHref();
	}
	
	public static List<OMObservationType> getObservations(SIGMETType sigmet) {
		List<OMObservationType> observations = new ArrayList<OMObservationType>();
		for (OMObservationPropertyType analysis : sigmet.getAnalysis()) {
			observations.add(analysis.getOMObservation());
		}
		if (sigmet instanceof TropicalCycloneSIGMETType) {
			observations.add(((TropicalCycloneSIGMETType) sigmet).getForecastPositionAnalysis().getOMObservation());
		}
		// TODO: implement VolcanicAshSIGMETType
		
		return observations;
	}
	
	public static List<RigidShape> getRigidShapes(SIGMETType sigmet, IwxxmUnmarshaller unmarshaller) throws JAXBException {
		List<RigidShape> rigidShapes = new ArrayList<RigidShape>();
		for (OMObservationPropertyType analysis : sigmet.getAnalysis()) {
			rigidShapes.addAll(OmData.getRigidShapes(analysis.getOMObservation(), unmarshaller));
		}
		return rigidShapes;
	}
	
	public static List<RigidShape> getRigidShapes(MeteorologicalPositionCollectionType mpc) {
		List<RigidShape> rigidShapes = new ArrayList<RigidShape>();
		for (MeteorologicalPositionPropertyType member : mpc.getMember()) {
			rigidShapes.addAll(IwxxmData.getRigidShapes(member.getMeteorologicalPosition()));
		}
		return rigidShapes;
	}
	
	public static List<RigidShape> getRigidShapes(MeteorologicalPositionType mp) {
		return AixmData.getRigidShapes(mp.getGeometry().getAirspaceVolume());
	}
	
	public static List<RigidShape> getRigidShapes(EvolvingMeteorologicalConditionType emc) {
		return AixmData.getRigidShapes(emc.getGeometry().getAirspaceVolume());
	}
	
	public static List<Airspace> getAirspaces(SIGMETType sigmet, IwxxmUnmarshaller unmarshaller) throws JAXBException {
		List<Airspace> airspaces = new ArrayList<Airspace>();
		for (OMObservationPropertyType analysis : sigmet.getAnalysis()) {
			airspaces.addAll(OmData.getAirspaces(analysis.getOMObservation(), unmarshaller));
		}
		return airspaces;
	}
	
	public static List<Airspace> getAirspaces(MeteorologicalPositionCollectionType mpc) {
		List<Airspace> airspaces = new ArrayList<Airspace>();
		for (MeteorologicalPositionPropertyType member : mpc.getMember()) {
			airspaces.addAll(IwxxmData.getAirspaces(member.getMeteorologicalPosition()));
		}
		return airspaces;
	}
	
	public static List<Airspace> getAirspaces(MeteorologicalPositionType mp) {
		return AixmData.getAirspaces(mp.getGeometry().getAirspaceVolume());
	}
	
	public static List<Airspace> getAirspaces(EvolvingMeteorologicalConditionType emc) {
		return AixmData.getAirspaces(emc.getGeometry().getAirspaceVolume());
	}
	
	public static Angle getDirectionOfMotion(EvolvingMeteorologicalConditionType emc) {
		return Angle.fromDegrees(
				Conversions.toDegrees(
						emc.getDirectionOfMotion().getValue(),
						emc.getDirectionOfMotion().getUom()));
	}
	
	public static double getSpeedOfMotion(EvolvingMeteorologicalConditionType emc) {
		return Conversions.toMetersPerSecond(
				emc.getSpeedOfMotion().getValue(),
				emc.getSpeedOfMotion().getUom());
	}
	
}
