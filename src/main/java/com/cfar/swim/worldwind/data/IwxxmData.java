package com.cfar.swim.worldwind.data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBException;

import com.cfar.swim.iwxxm.bind.IwxxmUnmarshaller;
import com.cfar.swim.worldwind.planning.TimeInterval;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.render.RigidShape;
import icao.iwxxm.EvolvingMeteorologicalConditionType;
import icao.iwxxm.MeteorologicalPositionCollectionType;
import icao.iwxxm.MeteorologicalPositionPropertyType;
import icao.iwxxm.MeteorologicalPositionType;
import icao.iwxxm.SIGMETType;
import icao.iwxxm.TAFType;
import net.opengis.om.OMObservationPropertyType;
import net.opengis.om.OMObservationType;

public class IwxxmData {
	
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
