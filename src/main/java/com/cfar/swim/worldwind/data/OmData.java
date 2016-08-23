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

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;

import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import com.cfar.swim.iwxxm.bind.IwxxmUnmarshaller;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.render.ObstaclePath;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.render.RigidShape;
import icao.iwxxm.EvolvingMeteorologicalConditionType;
import icao.iwxxm.MeteorologicalPositionCollectionType;
import icao.iwxxm.MeteorologicalPositionType;
import net.opengis.gml.TimeEdgeType;
import net.opengis.gml.TimeInstantType;
import net.opengis.gml.TimeNodeType;
import net.opengis.gml.TimePeriodType;
import net.opengis.gml.TimeTopologyComplexType;
import net.opengis.om.OMObservationType;

public class OmData {
	
	public static ZonedDateTime getPhenomenonTime(OMObservationType observation) {
		ZonedDateTime phenomenonTime = null;
		Object time = observation.getPhenomenonTime().getAbstractTimeObject();
		
		if (null != time) {
			time = ((JAXBElement<?>) time).getValue();
		}
		
		// TODO: implement other concrete types
		if (time instanceof TimeTopologyComplexType) {
			
		} else if (time instanceof TimeInstantType) {
			phenomenonTime = GmlData.getTime((TimeInstantType) time);
		} else if (time instanceof TimePeriodType) {
			
		} else if (time instanceof TimeEdgeType) {
			
		} else if (time instanceof TimeNodeType) {
			
		}
		
		return phenomenonTime;
	}
	
	public static ZonedDateTime getResultTime(OMObservationType observation) {
		return GmlData.getTime(observation.getResultTime().getTimeInstant());
	}
	
	public static TimeInterval getValidTimeInterval(OMObservationType observation) {
		TimeInterval validTimeInterval = null;
		
		// TODO: deal with xlink:href (annotations? @XmlIdRef,@XmlId, XmlAdapter?)
		if (null != observation.getValidTime().getTimePeriod()) {
			validTimeInterval = GmlData.getTimeInterval(observation.getValidTime().getTimePeriod());
		}
		
		return validTimeInterval;
	}
	
	// TODO: RigidShapes and Paths should be TimedRenderable and ThresholdRenderable!
	public static List<RigidShape> getRigidShapes(OMObservationType observation, IwxxmUnmarshaller unmarshaller) throws JAXBException {
		List<RigidShape> rigidShapes = new ArrayList<RigidShape>();
		
		Node node = (Node) observation.getResult();
		if (node.hasChildNodes()) {
			NodeList children = node.getChildNodes();
			int length = children.getLength();
			for (int i = 0; i < length; i++) {
				Node child = children.item(i);
				
				if (Node.ELEMENT_NODE == child.getNodeType()) {
					JAXBElement<?> content = (JAXBElement<?>) unmarshaller.unmarshal(child);
					Object object = content.getValue();
					if (object instanceof MeteorologicalPositionCollectionType) {
						rigidShapes.addAll(IwxxmData.getRigidShapes((MeteorologicalPositionCollectionType) object));
					} else if (object instanceof MeteorologicalPositionType) {
						rigidShapes.addAll(IwxxmData.getRigidShapes((MeteorologicalPositionType) object));
					} else if (object instanceof EvolvingMeteorologicalConditionType) {
						rigidShapes.addAll(IwxxmData.getRigidShapes((EvolvingMeteorologicalConditionType) object));
					}
				}
			}
		}
		
		return rigidShapes;
	}
	
	public static Path getObservationPath(OMObservationType observation, IwxxmUnmarshaller unmarshaller) throws JAXBException {
		List<RigidShape> rigidShapes = OmData.getRigidShapes(observation, unmarshaller);
		return OmData.getObservationPath(rigidShapes);
	}
	
	public static Path getObservationPath(List<RigidShape> rigidShapes) {
		List<Position> positions = new ArrayList<Position>();
		
		for (RigidShape rigidShape : rigidShapes) {
			positions.add(rigidShape.getCenterPosition());
		}
		
		return new ObstaclePath(positions);
	}
	
}
