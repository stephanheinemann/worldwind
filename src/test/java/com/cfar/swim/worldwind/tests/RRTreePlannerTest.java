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
package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.Date;
import java.util.Set;

import org.junit.Test;
import org.xml.sax.InputSource;

import com.cfar.swim.worldwind.ai.rrt.adrrt.ADRRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.ai.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.hrrt.Heuristic;
import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.CoordinateTransformations;
import com.cfar.swim.worldwind.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.render.Obstacle;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Matrix;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * @author Manuel Rosa
 *
 */
public class RRTreePlannerTest {

	static final int REPETITIONS = 30;
	Iris iris;
	A320 a320;
	Globe globe;
	SamplingEnvironment samplingEnv;
	Position origin, destination;
	ZonedDateTime etd;

	@Test
	public void RRTreeTester() {

		// Set inputs for planner
		this.setScenario();
		
//		this.embedObstacle();
		
		System.out.println("Repetitions #" + REPETITIONS);
		
		/*
		Position positionNew, positionNewT, positionRand;
		RRTreePlanner plannerRRT = new RRTreePlanner(a320, samplingEnv, 250, 5, 1500);
		for (int i = 0; i < REPETITIONS; i++) {
			origin = new Position(Angle.fromDegrees(30), Angle.fromDegrees(20), 0);
			positionRand = new Position(Angle.fromDegrees(origin.latitude.degrees+0.000),
					Angle.fromDegrees(origin.longitude.degrees+0.000), origin.elevation+1000);
			positionRand = samplingEnv.sampleRandomPosition();
			positionNew = plannerRRT.growPosition(positionRand, origin);
			positionNewT = this.growPositionENU(positionRand, origin, plannerRRT.getAircraft(), plannerRRT.getEPSILON());
			System.out.println("Rand: "+positionRand+"\nNear: "+origin+
					"\nNew:  "+positionNew +"\t"+samplingEnv.getDistance(origin, positionNew)+
							"\t"+a320.getCapabilities().isFeasible(origin, positionNew, globe)+
					"\nNewT: "+positionNewT+"\t"+samplingEnv.getDistance(origin, positionNewT)+
							"\t"+a320.getCapabilities().isFeasible(origin, positionNewT, globe)+"\n");
		}
		*/
		
		// Basic RRTreePlanner
		// Bias 5%
		this.basicRRTreeTester(250, 5, Strategy.EXTEND);
		/*
		// Bias 1%
		this.basicRRTreeTester(250, 1, Strategy.EXTEND);

		System.out.println();
		// Heuristic RRTreePlanner
		this.heuristicRRTreeTester(250, 5, .9, 5, Heuristic.hRRT);
		this.heuristicRRTreeTester(250, 5, .9, 5, Heuristic.IkRRT);
		this.heuristicRRTreeTester(250, 5, .9,  5, Heuristic.BkRRT);
		 */
		
		/*
		// Anytime Dynamic
		this.anydynRRTreeTester(250, 5, 0d, 1d, 0.05);
		 */
		System.out.println();
	}

	public void setScenario() {
		// Create box area in globe
		globe = new Earth();
		Sector cadboroBay = new Sector(
				Angle.fromDegrees(48.44),
				Angle.fromDegrees(48.46),
				Angle.fromDegrees(-123.29),
				Angle.fromDegrees(-123.27));
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, cadboroBay, 0.0, 1500.0);

		// Create environment from box
		samplingEnv = new SamplingEnvironment(new Box(boxNASA));
		samplingEnv.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(48.445, -123.285, 10);
		destination = Position.fromDegrees(48.455, -123.275, 1000);
		etd = ZonedDateTime.of(LocalDate.of(2018, 5, 1), LocalTime.of(22, 0), ZoneId.of("UTC"));
		iris = new Iris(origin, 500, CombatIdentification.FRIEND);
		a320 = new A320(origin, 5000, CombatIdentification.FRIEND);
	}

	public void basicRRTreeTester(double epsilon, int bias, Strategy strategy) {
		System.out.println(String.format("\tBasic RRTreeTester - %s - e=%.1f b=%d", strategy, epsilon, bias));

		RRTreePlanner plannerRRT = new RRTreePlanner(iris, samplingEnv, epsilon, bias, 1500);
		plannerRRT.setStrategy(strategy);

		Path path;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;
		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {
			t0 = System.currentTimeMillis();
			path = plannerRRT.plan(origin, destination, etd);
			size = Iterables.size(path.getPositions());
			waypoints = plannerRRT.getWaypointList().size();
			cost = plannerRRT.getGoal().getCost();
			time = System.currentTimeMillis() - t0;
			this.log(size, waypoints, cost, time);
			sizeT += size; waypointsT += waypoints; costT+= cost; timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	public void heuristicRRTreeTester(double epsilon, int bias, double prob, int nbr, Heuristic heuristic) {
		System.out.println(String.format("\tHeuristic RRTreeTester - %s - e=%.1f b=%d p=%.2f n=%d",
				heuristic, epsilon,	bias, prob, nbr));

		HRRTreePlanner plannerHRRT = new HRRTreePlanner(iris, samplingEnv, epsilon, bias, 1500, prob, nbr);
		plannerHRRT.setHeuristic(heuristic);

		Path path;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;
		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {
			t0 = System.currentTimeMillis();
			path = plannerHRRT.plan(origin, destination, etd);
			size = Iterables.size(path.getPositions());
			waypoints = plannerHRRT.getWaypointList().size();
			cost = plannerHRRT.getGoal().getCost();
			time = System.currentTimeMillis() - t0;
			this.log(size, waypoints, cost, time);
			sizeT += size; waypointsT += waypoints; costT+= cost; timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	public void processData(double size, double waypoints, double cost, double time) {
		size = size / REPETITIONS;
		waypoints = waypoints / REPETITIONS;
		cost = cost / REPETITIONS;
		time = time / REPETITIONS;

		System.out.println(String.format("Waypoints created: %.1f Path size: %.1f Cost: %.4f Time: %.1f(ms)",
				waypoints, size, cost, time));
		assertTrue("RRTree should find a path", size > 0);
	}
	
	public void log(double size, double waypoints, double cost, double time) {
		System.out.println(String.format("%.1f, %.1f, %.4f, %.1f", waypoints, size, cost, time));
	}
	
	public void anydynRRTreeTester(double epsilon, int bias, double init, double fina, double imp) {
		System.out.println(String.format("\tAnytimeDynamic RRTreeTester - () - e=%.1f b=%d cb_i=%.2f cb_f=%.2f imp=%.2f",
				 epsilon, bias, init, fina, imp));

		ADRRTreePlanner plannerADRRT = new ADRRTreePlanner(iris, samplingEnv, epsilon, bias, 1500);
		plannerADRRT.setMinimumQuality(init);
		plannerADRRT.setMaximumQuality(fina);
		plannerADRRT.setQualityImprovement(imp);

		Path path;
		double size = 0, waypoints = 0, cost = 0d;
		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {
			path = plannerADRRT.plan(origin, destination, etd);
			size += Iterables.size(path.getPositions());
			waypoints += plannerADRRT.getWaypointList().size();
			cost += plannerADRRT.getGoal().getCost();
		}
	}
	
	public void embedObstacle() {
		assertTrue(samplingEnv.getObstacles().isEmpty());
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(new File("src/test/resources/xml/iwxxm/sigmet-victoria-ts.xml"))));
			for (Obstacle obstacle : obstacles) {
				samplingEnv.embed(obstacle);
			}
			assertTrue(!samplingEnv.getObstacles().isEmpty());
		} catch (Exception e) {
			e.printStackTrace();
		}
		samplingEnv.setTime(etd);
	}

	
	public Position growPositionENU0(Position positionRand, Position positionNear, Aircraft acft, double EPSILON) {
		Position positionNew;
		System.out.println("");
		
		// Compute point in NASA ECEF from position
		Vec4 pointNear = globe.computePointFromPosition(positionNear);
		Vec4 pointRand = globe.computePointFromPosition(positionRand);
//		System.out.println("!!Near: "+pointNear+" !!Rand: "+pointRand);
		// Transform from NASA ECEF to standard ECEF
		pointNear = new Vec4(pointNear.z, pointNear.x, pointNear.y);
		pointRand = new Vec4(pointRand.z, pointRand.x, pointRand.y);
		System.out.println("**Near: "+pointNear+" **Rand: "+pointRand);

		// Save lat and lon from reference point
		Angle lat = positionNear.latitude;
		Angle lon = positionNear.longitude;
		System.out.println("Lat: "+lat+" Lon: "+lon);
		
		
		
		
		// Transform from standard ECEF to ENU in position Near
		Angle angleZ = Angle.POS90.add(lon), angleX = Angle.POS90.subtract(lat);
		System.out.println("AngleX: "+angleX+" AngleZ: "+angleZ);
		
		Vec4 pointENU = pointRand.subtract3(pointNear);
		double x = pointENU.x, y = pointENU.y, z = pointENU.z;
		double cLat=angleX.cos(), sLat=angleX.sin();
		double cLon=angleZ.cos(), sLon=angleZ.sin();
		
		double e = x*cLon + y*sLon;
		double n = -x*sLon*cLat + y*cLon*cLat + z*sLat;
		double u = x*sLon*sLat - y*cLon*sLat + z*cLat;

		pointENU = new Vec4(e, n, u);
		System.out.println("ENU: "+pointENU);
		
		
		
		
		
		
		
		// Calculate azimuth and elevation
		Angle azi = Angle.fromRadians(Math.atan2(e, n));
		Angle ele = Angle.fromRadians(Math.atan2(u, Math.sqrt(n*n + e*e)));
		System.out.println("??Ele: "+ele+" Azi: "+azi);
		
		// Get angles defining feasibility region
		Angle maxEle = Angle.fromRadians(Math.asin(acft.getCapabilities().getMaximumRateOfClimb()/
				acft.getCapabilities().getMaximumRateOfClimbSpeed())).multiply(0.9);
		Angle minEle = Angle.fromRadians(Math.asin(acft.getCapabilities().getMaximumRateOfDescent()/
				acft.getCapabilities().getMaximumRateOfDescentSpeed())).multiply(-0.9);
		
		double distance = samplingEnv.getDistance(positionNear, positionRand);

		// Non-feasible region
		if (ele.compareTo(maxEle) == 1 || ele.compareTo(minEle) == -1) {
			System.out.println("Not feasible -> " + ele);
			// Saturate elevation and distance to maximum values
			ele = ele.compareTo(maxEle) == 1 ? maxEle : minEle;
			distance = distance > EPSILON ? EPSILON : distance;

			// Calculate maximum feasible point in ENU
			u = distance * Math.sin(ele.radians);
			e = distance * Math.cos(ele.radians) * Math.sin(azi.radians);
			n = distance * Math.cos(ele.radians) * Math.cos(azi.radians);
			pointENU = new Vec4(e, n, u);
			System.out.println("ENU: " + pointENU);

			// Transform from ENU to standard ECEF
			angleX = Angle.NEG90.add(lat);
			angleZ = Angle.NEG90.subtract(lon);
			System.out.println("@ngleX: "+angleX+" @ngleZ: "+angleZ);
//			Vec4 pointNew = pointENU.transformBy3(Matrix.fromRotationX(angleX))
//					.transformBy3(Matrix.fromRotationZ(angleZ)).add3(pointNear);
			cLat=angleX.cos(); sLat=angleX.sin();
			cLon=angleZ.cos(); sLon=angleZ.sin();
			x =  e*cLon + n*cLat*sLon + u*sLat*sLon;
			y = -e*sLon + n*cLat*cLon + u*sLat*cLon;
			z = -n*sLat + u*cLat;
			
			Vec4 pointNew = new Vec4(x,y,z);
			pointNew = pointNew.add3(pointNear);
			System.out.println("PointNew: "+pointNew);
			System.out.println("PointNe@r: "+pointNear);
			
			
			
			
			// Transform from standard ECEF to NASA ECEF
			pointNew = new Vec4(pointNew.y, pointNew.z, pointNew.x);
			// Compute position from NASA ECEF
			positionNew = globe.computePositionFromPoint(pointNew);
			System.out.println("PositionNew: " + positionNew);
			
			// Calculate azimuth and elevation (DEBUGGING)
			pointNew = new Vec4(pointNew.z, pointNew.x, pointNew.y);
			angleZ = Angle.POS90.add(lon); angleX = Angle.POS90.subtract(lat);
			pointENU = pointNew.subtract3(pointNear).transformBy3(Matrix.fromRotationZ(angleZ)).transformBy3(Matrix.fromRotationX(angleX));
			e = pointENU.x; n = pointENU.y; u = pointENU.z;
			azi = Angle.fromRadians(Math.atan2(e, n));
			ele = Angle.fromRadians(Math.atan2(u, Math.sqrt(n*n + e*e)));
			System.out.println("**Ele: "+ele+" Azi: "+azi);
			
			// Height and Distance (DEBUGGING)
			double distance2D = LatLon.linearDistance(positionNear, positionNew).getRadians() * globe.getRadius();
			double height = positionNew.getElevation() - positionNear.getElevation();
			double slt, maxSlt;
			if(height>0) {
				slt = acft.getCapabilities().getMaximumRateOfClimbSpeed()/acft.getCapabilities().getMaximumRateOfClimb() *height;
				maxSlt =  Math.sqrt(Math.pow(distance2D, 2) + Math.pow(height, 2));
			}
			else {
				slt = acft.getCapabilities().getMaximumRateOfDescentSpeed()/acft.getCapabilities().getMaximumRateOfDescent() *height;
				maxSlt =  Math.sqrt(Math.pow(distance2D, 2) + Math.pow(height, 2));
			}
			System.out.println("Height: "+height+" Distance2D: "+distance2D+" Slant: "+slt+" MaxSlt: "+maxSlt);
		}
		// Feasible Region
		else {
			System.out.println("Feasible -> "+ele);
			if(distance<=EPSILON) {
				positionNew = positionRand;
			}
			else {
				// Copy from basic RRT
				positionNew = positionRand;
			}
		}
		
		System.out.println("");
		return positionNew;
	}
	
	public Position growPositionENU(Position positionRand, Position positionNear, Aircraft acft, double EPSILON) {
		Position positionNew;

		// Transform position to ENU coordinates 
		Vec4 pointENU = CoordinateTransformations.llh2enu(positionNear, positionRand, globe);
		
		// Calculate azimuth and elevation
		double e = pointENU.x, n = pointENU.y, u = pointENU.z;
		Angle azi = Angle.fromRadians(Math.atan2(e, n));
		Angle ele = Angle.fromRadians(Math.atan2(u, Math.sqrt(n*n + e*e)));
		
		// Get angles defining feasibility region
		Angle maxEle = Angle.fromRadians(Math.asin(acft.getCapabilities().getMaximumRateOfClimb()/
				acft.getCapabilities().getMaximumRateOfClimbSpeed())).multiply(1);
		Angle minEle = Angle.fromRadians(Math.asin(acft.getCapabilities().getMaximumRateOfDescent()/
				acft.getCapabilities().getMaximumRateOfDescentSpeed())).multiply(-1);
		
		double distance = samplingEnv.getDistance(positionNear, positionRand);

		// Non-feasible region
		if (ele.compareTo(maxEle) == 1 || ele.compareTo(minEle) == -1) {
			System.out.println("Not feasible -> " + ele);
			
			// Saturate elevation and distance to maximum values
			ele = ele.compareTo(maxEle) == 1 ? maxEle : minEle;
			distance = distance > EPSILON ? EPSILON : distance;

			// Calculate maximum feasible point in ENU
			u = distance * Math.sin(ele.radians);
			e = distance * Math.cos(ele.radians) * Math.sin(azi.radians);
			n = distance * Math.cos(ele.radians) * Math.cos(azi.radians);
			pointENU = new Vec4(e, n, u);

			// Transform from ENU to standard ECEF
			positionNew = CoordinateTransformations.enu2llh(positionNear, pointENU, globe);
		}
		// Feasible Region
		else {
			System.out.println("Feasible -> "+ele);
			
			if(distance<=EPSILON) {
				positionNew = positionRand;
			}
			else {
				// Copy from basic RRT
				positionNew = positionRand;
			}
		}
		
		return positionNew;
	}
	
}
