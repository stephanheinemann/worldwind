package com.cfar.swim.worldwind.data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;

import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import com.cfar.swim.iwxxm.bind.IwxxmUnmarshaller;

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
		ZonedDateTime phenomenonTime = ZonedDateTime.now();
		Object time = observation.getPhenomenonTime().getAbstractTimeObject().getValue();
		
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
	
}
