/**
 * Copyright (c) 2018, Manuel Rosa (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.geom;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;

/**
 * @author Manuel Rosa
 *
 */
public class CoordinateTransformations {

	/**
	 * Converts a position in Earth Centered Earth Fixed frame, in accordance with
	 * NASA's WW orientation, to the local frame East North Up centered in the
	 * reference position.
	 * 
	 * @param reference
	 * @param other
	 * @param globe
	 * 
	 * @return
	 */
	public static Vec4 llh2enu(Position reference, Position other, Globe globe) {
		// TODO: Correct computational errors with small values instead of zero
		// TODO: Investigate use of NASA.Matrix

		// Compute point in NASA ECEF from position
		Vec4 pointNear = globe.computePointFromPosition(reference);
		Vec4 pointRand = globe.computePointFromPosition(other);

		// Transform from NASA ECEF to standard ECEF
		pointNear = new Vec4(pointNear.z, pointNear.x, pointNear.y);
		pointRand = new Vec4(pointRand.z, pointRand.x, pointRand.y);

		// Save lat and lon from reference point
		Angle lat = reference.latitude;
		Angle lon = reference.longitude;

		// Transform from standard ECEF to ENU in position Near
		// Translation
		Vec4 pointENU = pointRand.subtract3(pointNear);
		// Rotation
		Angle angleZ = Angle.POS90.add(lon), angleX = Angle.POS90.subtract(lat);

		double x = pointENU.x, y = pointENU.y, z = pointENU.z;
		double cLat = angleX.cos(), sLat = angleX.sin();
		double cLon = angleZ.cos(), sLon = angleZ.sin();

		double e = x * cLon + y * sLon;
		double n = -x * sLon * cLat + y * cLon * cLat + z * sLat;
		double u = x * sLon * sLat - y * cLon * sLat + z * cLat;

		pointENU = new Vec4(e, n, u);

		return pointENU;
	}

	/**
	 * Converts a position in the local frame East North Up centered in the
	 * reference position to Earth Centered Earth Fixed frame, in accordance with
	 * NASA's WW orientation.
	 * 
	 * @param reference
	 * @param pointENU
	 * @param globe
	 * 
	 * @return
	 */
	public static Position enu2llh(Position reference, Vec4 pointENU, Globe globe) {
		// TODO: Correct computational errors with small values instead of zero
		// TODO: Investigate use of NASA.Matrix

		// Compute point in NASA ECEF from position
		Vec4 pointNear = globe.computePointFromPosition(reference);

		// Transform from NASA ECEF to standard ECEF
		pointNear = new Vec4(pointNear.z, pointNear.x, pointNear.y);

		// Save lat and lon from reference point
		Angle lat = reference.latitude;
		Angle lon = reference.longitude;

		// Transform from ENU to standard ECEF
		// Rotation
		Angle angleX = Angle.NEG90.add(lat);
		Angle angleZ = Angle.NEG90.subtract(lon);

		double e = pointENU.x, n = pointENU.y, u = pointENU.z;
		double cLat = angleX.cos(), sLat = angleX.sin();
		double cLon = angleZ.cos(), sLon = angleZ.sin();

		double x = e * cLon + n * cLat * sLon + u * sLat * sLon;
		double y = -e * sLon + n * cLat * cLon + u * sLat * cLon;
		double z = -n * sLat + u * cLat;
		Vec4 point = new Vec4(x, y, z);

		// Translation
		point = point.add3(pointNear);

		// Transform from standard ECEF to NASA ECEF
		point = new Vec4(point.y, point.z, point.x);

		// Compute position from NASA ECEF
		Position position = globe.computePositionFromPoint(point);

		return position;
	}

	/**
	 * Computes the Azimuth, Elevation and Range of a given point expressed in ENU
	 * coordinates to the same reference.
	 * 
	 * @param pointENU the ENU coordinates
	 * 
	 * @return AER coordinates
	 */
	public static Vec4 enu2aer(Vec4 pointENU) {
		double e = pointENU.x, n = pointENU.y, u = pointENU.z;

		double azimuth = Math.atan2(e, n);
		double elevation = Math.atan2(u, Math.sqrt(n * n + e * e));
		double range = Math.sqrt(e * e + n * n + u * u);

		return new Vec4(azimuth, elevation, range);
	}

	/**
	 * Computes the Azimuth, Elevation and Range of a given position in LLH
	 * coordinates relative to another reference position.
	 * 
	 * @param reference
	 * @param other
	 * @param globe
	 * 
	 * @return
	 */
	public static Vec4 llh2aer(Position reference, Position other, Globe globe) {
		Vec4 pointENU = CoordinateTransformations.llh2enu(reference, other, globe);

		return CoordinateTransformations.enu2aer(pointENU);
	}
	
	/**
	 * 
	 * @param point
	 * @param angle
	 * @return
	 */
	public static Vec4 rotationZ(Vec4 point, double angle) {
		double c = Math.cos(angle);
		double s = Math.sin(angle);
		
		double x =  point.x * c + point.y * s;
		double y = -point.x * s + point.y * c;
		double z =  point.z;
		
		return new Vec4(x, y, z);
	}
	
	/**
	 * 
	 * @param point
	 * @param angle
	 * @return
	 */
	public static Vec4 rotationX(Vec4 point, double angle) {
		double c = Math.cos(angle);
		double s = Math.sin(angle);
		
		double x =  point.x;
		double y =  point.y * c + point.z * s;
		double z = -point.y * s + point.z * c;
		
		return new Vec4(x, y, z);
	}
}
