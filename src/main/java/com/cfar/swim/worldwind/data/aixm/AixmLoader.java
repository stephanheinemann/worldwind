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
package com.cfar.swim.worldwind.data.aixm;

import java.time.ZonedDateTime;
import java.util.HashSet;
import java.util.Set;

import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;

import org.xml.sax.InputSource;

import com.cfar.swim.aixm.bind.AixmUnmarshaller;
import com.cfar.swim.worldwind.data.Conversions;
import com.cfar.swim.worldwind.data.ObstacleLoader;
import com.cfar.swim.worldwind.data.SwimProtocol;
import com.cfar.swim.worldwind.data.SwimResource;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.airspaces.ObstacleBox;

import aero.aixm.AirspaceGeometryComponentType;
import aero.aixm.AirspaceTimeSlicePropertyType;
import aero.aixm.AirspaceType;
import aero.aixm.AirspaceVolumeType;
import aero.aixm.CodeVerticalReferenceType;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.LatLon;
import net.opengis.gml.AbstractCurveSegmentType;
import net.opengis.gml.AbstractTimePrimitiveType;
import net.opengis.gml.DirectPositionListType;
import net.opengis.gml.GeodesicStringType;
import net.opengis.gml.TimePeriodType;

/**
 * Realizes a AIXM obstacle loader.
 * 
 * @author Stephan Heinemann
 *
 */
public class AixmLoader implements ObstacleLoader {
	
	// NOTE: this is only a very simplified, specialized and rudimentary loader
	
	// TODO: cost and separation could be part of FIXM extension
	// Special Use Airspace, Restricted Airspace, Airspace Classification, Airspace Cost Map
	
	/** the default cost of an aircraft obstacle */
	private static final double DEFAULT_AIRSPACE_COST = 100d;
	
	/** the AIXM unmarshaller of this AIXM loader */
	private AixmUnmarshaller unmarshaller;
	
	/**
	 * Constructs a new AIXM obstacle loader.
	 * 
	 * @throws JAXBException if the AIXM obstacle loader cannot be constructed
	 */
	public AixmLoader() throws JAXBException {
		this.unmarshaller = new AixmUnmarshaller();
	}
	
	/**
	 * Loads AIXM obstacles from a SWIM resource.
	 * 
	 * @param resource the SWIM resource
	 * 
	 * @return the loaded obstacles
	 * 
	 * @see ObstacleLoader#load(SwimResource)
	 */
	@Override
	public Set<Obstacle> load(SwimResource resource) {
		Set<Obstacle> obstacles = new HashSet<>();
		
		if (resource.getProtocol().isEmpty()
				|| (resource.getProtocol().isPresent()
				&& resource.getProtocol().get().equals(SwimProtocol.AIXM))) {
			try {
				obstacles = this.load(new InputSource(resource.getResource().toURL().openStream()));
			} catch (Exception e) {
				//e.printStackTrace();
			}
		}
		
		return obstacles;
	}
	
	/**
	 * Loads AIXM obstacles from an input source.
	 * 
	 * @param source the input source
	 * 
	 * @return the loaded AIXM obstacles
	 * 
	 * @throws JAXBException if the AIXM obstacles cannot be loaded
	 */
	public Set<Obstacle> load(InputSource source) throws JAXBException {
		Set<Obstacle> obstacles = new HashSet<>();
		JAXBElement<?> aixmElement = (JAXBElement<?>) this.unmarshaller.unmarshal(source);
		
		if (aixmElement.getValue() instanceof AirspaceType) {
			obstacles = this.loadAirspace((AirspaceType) aixmElement.getValue());
		}
		
		return obstacles;
	}
	
	/**
	 * Loads AIXM airspace obstacles.
	 * 
	 * @param airspace the AIXM airspace
	 * 
	 * @return the loaded AIXM airspace obstacles
	 */
	private Set<Obstacle> loadAirspace(AirspaceType airspace) {
		Set<Obstacle> obstacles = new HashSet<>();
		
		for (AirspaceTimeSlicePropertyType airspaceTimeSlice : airspace.getTimeSlice()) {
			if ((null != airspaceTimeSlice.getAirspaceTimeSlice().getGeometryComponent())
					&& !airspaceTimeSlice.getAirspaceTimeSlice().getGeometryComponent().isEmpty()) {
				AirspaceGeometryComponentType airspaceGeometryComponent =
						airspaceTimeSlice.getAirspaceTimeSlice().getGeometryComponent().get(0)
						.getAirspaceGeometryComponent();
				if (null != airspaceGeometryComponent.getTheAirspaceVolume()) {
					AirspaceVolumeType airspaceVolume = airspaceGeometryComponent.getTheAirspaceVolume().getAirspaceVolume();
					if ((null != airspaceVolume.getCentreline())
							&& (null != airspaceVolume.getWidth())
							&& (null != airspaceVolume.getLowerLimit())
							&& (null != airspaceVolume.getUpperLimit())) {
						
						for (JAXBElement<? extends AbstractCurveSegmentType> segment :
							airspaceVolume.getCentreline().getValue().getCurve().getValue()
							.getSegments().getAbstractCurveSegment()) {
							// TODO: connected boxes versus several boxes (complex airspaces)
							if (GeodesicStringType.class == segment.getDeclaredType()) {
								DirectPositionListType positions = ((GeodesicStringType) segment.getValue()).getPosList();
								if (null != positions && (4 == positions.getValue().size())) {
									LatLon begin = LatLon.fromDegrees(positions.getValue().get(0), positions.getValue().get(1));
									LatLon end = LatLon.fromDegrees(positions.getValue().get(2), positions.getValue().get(3));
								
									double width = Conversions.toMeters(
											airspaceVolume.getWidth().getValue().getValue().doubleValue(),
											airspaceVolume.getWidth().getValue().getUom());
									double lowerLimit = Conversions.toMeters(
											Double.parseDouble(airspaceVolume.getLowerLimit().getValue().getValue()),
											airspaceVolume.getLowerLimit().getValue().getUom());
									double upperLimit = Conversions.toMeters(
											Double.parseDouble(airspaceVolume.getUpperLimit().getValue().getValue()),
											airspaceVolume.getUpperLimit().getValue().getUom());
									
									ObstacleBox obstacleBox = new ObstacleBox(begin, end, width, width, lowerLimit, upperLimit);
									
									String lowerLimitReference = AVKey.ABOVE_MEAN_SEA_LEVEL;
									String upperLimitReference = AVKey.ABOVE_MEAN_SEA_LEVEL;
									
									if (null != airspaceVolume.getLowerLimitReference()) {
										lowerLimitReference = this.loadVerticalReference(airspaceVolume.getLowerLimitReference());
									}
									if (null != airspaceVolume.getUpperLimitReference()) {
										upperLimitReference = this.loadVerticalReference(airspaceVolume.getUpperLimitReference());
									}
									obstacleBox.setAltitudeDatum(lowerLimitReference, upperLimitReference);
									
									/*
									AviationZone zone = new AviationZone("GFGPAAR----AUSX");
									zone.setPositions();
									zone.setText("");
									zone.setModifier(SymbologyConstants.DATE_TIME_GROUP, Arrays.asList("180500Z", "180615Z"));
									zone.setModifier(SymbologyConstants.ALTITUDE_DEPTH, Arrays.asList("2000 FT AGL", "3000 FT AGL"));
									obstacleBox.setDepiction(new Depiction(zone));
									obstacleBox.getDepiction().setAnnotation(new DepictionAnnotation("", obstacleBox.getCenter()));
									*/
									
									obstacles.add(obstacleBox);
								}
							}
						}
					}
				}
			}
			
			TimeInterval timeInterval = this.loadValidTime(
					airspaceTimeSlice.getAirspaceTimeSlice().getValidTime().getAbstractTimePrimitive());
			double cost = AixmLoader.DEFAULT_AIRSPACE_COST;
			if (null != airspaceTimeSlice.getAirspaceTimeSlice().getLocalType()) {
				try {
				 cost = Double.parseDouble(airspaceTimeSlice.getAirspaceTimeSlice().getLocalType().getValue().getValue());
				} catch (NumberFormatException nfe) {
				}
			}
			CostInterval costInterval = new CostInterval(airspace.getId(), timeInterval, cost);
			// CostInterval costInterval = new CostInterval(airspaceTimeSlice.getAirspaceTimeSlice().getId(), timeInterval, cost);
			obstacles.stream().forEach(o -> o.setCostInterval(costInterval));
		}
		
		return obstacles;
	}
	
	/**
	 * Loads an AIXM vertical reference datum.
	 * 
	 * @param verticalReference the AIXM vertical reference datum
	 * 
	 * @return the loaded AIXM vertical reference datum
	 */
	private String loadVerticalReference(JAXBElement<CodeVerticalReferenceType> verticalReference) {
		String reference = AVKey.ABOVE_MEAN_SEA_LEVEL;
		
		if (verticalReference.getValue().getValue().equals("SFC")) {
			reference = AVKey.ABOVE_GROUND_LEVEL;
		} else if (verticalReference.getValue().getValue().equals("MSL")) {
			reference = AVKey.ABOVE_MEAN_SEA_LEVEL;
		} else if (verticalReference.getValue().getValue().equals("W84")) {
			reference = AVKey.ABOVE_MEAN_SEA_LEVEL;
		} else if (verticalReference.getValue().getValue().equals("STD")) {
			reference = AVKey.ABOVE_MEAN_SEA_LEVEL;
		} // AVKey.ABOVE_GROUND_LEVEL
		
		return reference;
	} 
	
	/**
	 * Loads an AIXM valid time.
	 * 
	 * @param validTime the AIXM valid time
	 * 
	 * @return the loaded AIXM valid time
	 */
	private TimeInterval loadValidTime(JAXBElement<? extends AbstractTimePrimitiveType> validTime) {
		TimeInterval timeInterval = new TimeInterval();
		
		if (TimePeriodType.class == validTime.getDeclaredType()) {
			if ((null != ((TimePeriodType) validTime.getValue()).getBeginPosition())
					&& (null != ((TimePeriodType) validTime.getValue()).getEndPosition())) {
				if ((0 < ((TimePeriodType) validTime.getValue()).getBeginPosition().getValue().size())
						&& (0 < ((TimePeriodType) validTime.getValue()).getEndPosition().getValue().size())) {
					
					timeInterval = new TimeInterval(
							ZonedDateTime.parse(((TimePeriodType) validTime.getValue()).getBeginPosition().getValue().get(0)),
							ZonedDateTime.parse(((TimePeriodType) validTime.getValue()).getEndPosition().getValue().get(0)));
				}
			}
		}
		
		return timeInterval;
	}
	
}
