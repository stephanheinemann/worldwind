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
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.render.Obstacle;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * @author Manuel Rosa
 *
 */
public class RRTreePlannerTest {

	static final int REPETITIONS = 0;
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
		
		Position positionNew, positionNewT, positionRand;
		RRTreePlanner plannerRRT = new RRTreePlanner(a320, samplingEnv, 250, 5, 1500);

		System.out.println("BoxOrig:\t"+globe.computePositionFromPoint(samplingEnv.getOrigin())+"\t"+samplingEnv.transformModelToBoxOrigin(samplingEnv.getOrigin()));
		System.out.println("Start:  \t"+origin+"\t"+samplingEnv.transformModelToBoxOrigin(globe.computePointFromPosition(origin)));
		positionRand = new Position(Angle.fromDegrees(origin.latitude.degrees+0.1),
									Angle.fromDegrees(origin.longitude.degrees+0.1), origin.elevation+500);
		System.out.println(".1|.1|500:\t"+positionRand+"\t"+samplingEnv.transformModelToBoxOrigin(globe.computePointFromPosition(positionRand)));
		positionRand = new Position(Angle.fromDegrees(origin.latitude.degrees+0.2),
				Angle.fromDegrees(origin.longitude.degrees+0.2), origin.elevation+500);
		System.out.println(".2|.2|500:\t"+positionRand+"\t"+samplingEnv.transformModelToBoxOrigin(globe.computePointFromPosition(positionRand)));
		
		for (int i = 0; i < REPETITIONS; i++) {
			positionRand = samplingEnv.sampleRandomPosition();
			positionNew = plannerRRT.growPosition(positionRand, origin);
			positionNewT = plannerRRT.growPositionTime(positionRand, origin);
			System.out.println("\nRand: "+positionRand+"\n\t"+samplingEnv.transformModelToBoxOrigin(globe.computePointFromPosition(positionRand))+
					"\nNear: "+origin+"\n\t"+samplingEnv.transformModelToBoxOrigin(globe.computePointFromPosition(origin))+
					"\nNew:  "+positionNew +"\t"+samplingEnv.getDistance(origin, positionNew)+
							"\t"+a320.getCapabilities().isFeasible(origin, positionNew, globe)+
							"\n\t"+samplingEnv.transformModelToBoxOrigin(globe.computePointFromPosition(positionNew))+
					"\nNewT: "+positionNewT+"\t"+samplingEnv.getDistance(origin, positionNewT)+
							"\t"+a320.getCapabilities().isFeasible(origin, positionNewT, globe)+
							"\n\t"+samplingEnv.transformModelToBoxOrigin(globe.computePointFromPosition(positionNewT)));
		}
		
		/*
		// Basic RRTreePlanner
		// Bias 5%
		this.basicRRTreeTester(250, 5, Strategy.EXTEND);
		this.basicRRTreeTester(250, 5, Strategy.CONNECT);
		// Bias 1%
		this.basicRRTreeTester(250, 1, Strategy.EXTEND);
		this.basicRRTreeTester(250, 1, Strategy.CONNECT);

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
		destination = Position.fromDegrees(48.455, -123.275, 100);
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

}
