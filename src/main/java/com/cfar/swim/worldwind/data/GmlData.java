package com.cfar.swim.worldwind.data;

import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.JAXBElement;

import com.cfar.swim.worldwind.planning.TimeInterval;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.render.AbstractSurfaceShape;
import gov.nasa.worldwind.render.SurfaceCircle;
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

public class GmlData {
	
	public static ZonedDateTime getTime(TimeInstantType timeInstant) {
		// TODO: check related time
		return GmlData.getTime(timeInstant.getTimePosition());
	}
	
	public static TimeInterval getTimeInterval(TimePeriodType timePeriod) {
		TimeInterval timeInterval = null;
		ZonedDateTime start = null;
		ZonedDateTime end = null;
		
		if (null != timePeriod.getBeginPosition()) {
			start = GmlData.getTime(timePeriod.getBeginPosition());
		} else if (null != timePeriod.getBegin()) {
			start = GmlData.getTime(timePeriod.getBegin().getTimeInstant().getTimePosition());
		}
		
		if (null != timePeriod.getEndPosition()) {
			end = GmlData.getTime(timePeriod.getEndPosition());
		} else if (null != timePeriod.getEnd()) {
			end = GmlData.getTime(timePeriod.getEnd().getTimeInstant().getTimePosition());
		}
		
		if ((null != start) && (null != end)) {
			timeInterval = new TimeInterval(start, end);
		}
		
		return timeInterval;
	}
	
	public static ZonedDateTime getTime(TimePositionType timePosition) {
		ZonedDateTime time = null;
		
		// TODO: check the rest of the list
		List<String> timeInstances = timePosition.getValue();
		if (!timeInstances.isEmpty()) {
			String timeInstance = timeInstances.get(0);
			// TODO: check other valid formats
			// 2012-08-25T16:00:00Z
			//time = ZonedDateTime.parse(timeInstance, DateTimeFormatter.ISO_INSTANT);
			time = ZonedDateTime.parse(timeInstance);
		}
		
		return time;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(AbstractSurfacePatchType patch) {
		AbstractSurfaceShape surfaceShape = null;
		
		// TODO: implement other concrete types
		if (patch instanceof ConeType) {
			
		} else if (patch instanceof CylinderType) {
			
		} else if (patch instanceof SphereType) {
			
		} else if (patch instanceof PolygonPatchType) {
			surfaceShape = GmlData.getSurfaceShape((PolygonPatchType) patch);
		} else if (patch instanceof RectangleType) {
			
		} else if (patch instanceof TriangleType) {
			
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(PolygonPatchType polygonPatch) {
		AbstractSurfaceShape surfaceShape = null;
		
		if (null != polygonPatch.getExterior()) {
			surfaceShape = GmlData.getSurfaceShape(polygonPatch.getExterior().getAbstractRing().getValue());
		}
		
		if (null != polygonPatch.getInterior()) {
			// TODO: implement interior shapes if required
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(AbstractRingType ring) {
		AbstractSurfaceShape surfaceShape = null;
		
		if (ring instanceof LinearRingType) {
			// TODO: implement linear ring type if required
		} else if (ring instanceof RingType) {
			surfaceShape = GmlData.getSurfaceShape((RingType) ring);
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(RingType ring) {
		AbstractSurfaceShape surfaceShape = null;
		List<AbstractSurfaceShape> curveShapes = new ArrayList<AbstractSurfaceShape>();
		
		List<CurvePropertyType> curves = ring.getCurveMember();
		for (CurvePropertyType curve : curves) {
			curveShapes.add(GmlData.getSurfaceShape(curve.getAbstractCurve().getValue()));
		}
		
		for (AbstractSurfaceShape curveShape : curveShapes) {
			// TODO: assemble curves to shape (check instances)
			if (curveShape instanceof SurfaceCircle) {
				surfaceShape = curveShape;
				break;
			}
		}
			
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(AbstractCurveType curve) {
		AbstractSurfaceShape surfaceShape = null;
		
		// TODO: implement other concrete types
		if (curve instanceof CompositeCurveType) {
			
		} else if (curve instanceof CurveType) {
			surfaceShape = GmlData.getSurfaceShape((CurveType) curve);
		} else if (curve instanceof LineStringType) {
			
		} else if (curve instanceof OrientableCurveType) {
			
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(CurveType curve) {
		AbstractSurfaceShape surfaceShape = null;
		
		for (JAXBElement<? extends AbstractCurveSegmentType> segment : curve.getSegments().getAbstractCurveSegment()) {
			surfaceShape = GmlData.getSurfaceShape(segment.getValue());
		}
		
		return surfaceShape;
	}
	
	public static AbstractSurfaceShape getSurfaceShape(AbstractCurveSegmentType segment) {
		AbstractSurfaceShape surfaceShape = null;
		
		// TODO: implement other concrete types
		if (segment instanceof CircleByCenterPointType) {
			surfaceShape = GmlData.getSurfaceCircle((CircleByCenterPointType) segment);
		} else if (segment instanceof ArcByCenterPointType) {
			
		} else if (segment instanceof ArcByBulgeType) {
		
		} else if (segment instanceof ArcStringByBulgeType) {
		
		} else if (segment instanceof CircleType) {
			
		} else if (segment instanceof ArcType) {
			
		} else if (segment instanceof ArcStringType) {
		
		} else if (segment instanceof BezierType) {	
		
		} else if (segment instanceof BSplineType) {
			
		} else if (segment instanceof ClothoidType) {
		
		} else if (segment instanceof CubicSplineType) {
		
		} else if (segment instanceof GeodesicType) {
		
		} else if (segment instanceof GeodesicStringType) {
			
		} else if (segment instanceof LineStringSegmentType) {
			
		} else if (segment instanceof OffsetCurveType) {
			
		}
		
		return surfaceShape;
	}
	
	public static SurfaceCircle getSurfaceCircle(CircleByCenterPointType circle) {
		LatLon center = null;
		
		// TODO: implement other possible elements
		if (null != circle.getPos()) {
			List<Double> position = circle.getPos().getValue();
			center = GmlData.getLocations(position).get(0);
		}
		
		double radius = Conversions.toMeters(
				circle.getRadius().getValue(), circle.getRadius().getUom());
		
		return new SurfaceCircle(center, radius);
	}
	
	public static List<LatLon> getLocations(List<Double> angles) {
		List<LatLon> locations = new ArrayList<LatLon>();
		Angle latitude = null;
		Angle longitude = null;
		
		int index = 0;
		for (Double angle : angles) {
			if (0 == index % 2) {
				latitude = Angle.fromDegrees(angle);
			} else {
				longitude = Angle.fromDegrees(angle);
				locations.add(new LatLon(latitude, longitude));
			}
			index++;
		}
		
		return locations;
	}
	
}
