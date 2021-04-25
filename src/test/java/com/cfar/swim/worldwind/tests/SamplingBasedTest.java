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
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Set;

import org.junit.Test;
import org.xml.sax.InputSource;

import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryMode;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryPlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.ai.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.hrrt.Heuristic;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.render.Obstacle;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * @author Manuel Rosa
 *
 */
public class SamplingBasedTest {

	static final int REPETITIONS = 200;
	Iris iris;
	SamplingEnvironment samplingEnv;
	Position origin, destination;
	ArrayList<Position> originList = new ArrayList<Position>(), destinationList = new ArrayList<Position>();
	ZonedDateTime etd;
	
	@Test
	public void SBTester() {

		// Set inputs for planner
		this.setScenario();

		this.basicPRMTester(300, 20, 250, QueryPlanner.FAS);
		this.heuristicRRTreeTester(250, 5, .9,  5, Heuristic.BkRRT);
		this.embedObstacle();
	}

	public void setScenario() {
		// Create box area in globe
		Globe globe = new Earth();
		Sector cadboroBay = new Sector(
				Angle.fromDegrees(48.44),
				Angle.fromDegrees(48.46),
				Angle.fromDegrees(-123.29),
				Angle.fromDegrees(-123.27));
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, cadboroBay, 0.0, 150.0);

		// Create environment from box
		samplingEnv = new SamplingEnvironment(new Box(boxNASA));
		samplingEnv.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(48.445, -123.285, 10);
		destination = Position.fromDegrees(48.455, -123.275, 100);
		Position position;
		for(int i=0; i<REPETITIONS; i++) {
			position = samplingEnv.sampleRandomPosition();
			while(samplingEnv.checkConflict(position, iris)) {
				position = samplingEnv.sampleRandomPosition();
			}
			originList.add(position);
			
			position = samplingEnv.sampleRandomPosition();
			while(samplingEnv.checkConflict(position, iris)) {
				position = samplingEnv.sampleRandomPosition();
			}
			destinationList.add(position);
		}
		etd = ZonedDateTime.of(LocalDate.of(2018, 5, 1), LocalTime.of(22, 0), ZoneId.of("UTC"));
		iris = new Iris(origin, 5000, CombatIdentification.FRIEND);
	}
	
	public void heuristicRRTreeTester(double epsilon, int bias, double prob, int nbr, Heuristic heuristic) {
		System.out.println(String.format("\tHeuristic RRTreeTester - %s - e=%.1f b=%d p=%.2f n=%d",
				heuristic, epsilon,	bias, prob, nbr));

		HRRTreePlanner plannerHRRT = new HRRTreePlanner(iris, samplingEnv, epsilon, bias, 1500, Strategy.EXTEND, Extension.LINEAR, prob, nbr);
		plannerHRRT.setHeuristic(heuristic);

		Path path;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;
		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {
			t0 = System.currentTimeMillis();
			path = plannerHRRT.plan(originList.get(i), destinationList.get(i), etd);
			size = Iterables.size(path.getPositions());
			waypoints = plannerHRRT.getWaypointList().size();
			cost = plannerHRRT.getGoal().getCost();
			time = System.currentTimeMillis() - t0;
			sizeT += size; waypointsT += waypoints; costT+= cost; timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}
	
	public void basicRRTreeTester(double epsilon, int bias, Strategy strategy) {
		System.out.println(String.format("\tBasic RRTreeTester - %s - e=%.1f b=%d", strategy, epsilon, bias));

		RRTreePlanner plannerRRT = new RRTreePlanner(iris, samplingEnv, epsilon, bias, 1500, strategy, Extension.LINEAR);

		Path path;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;
		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {
			t0 = System.currentTimeMillis();
			path = plannerRRT.plan(originList.get(i), destinationList.get(i), etd);
			size = Iterables.size(path.getPositions());
			waypoints = plannerRRT.getWaypointList().size();
			cost = plannerRRT.getGoal().getCost();
			time = System.currentTimeMillis() - t0;
			sizeT += size; waypointsT += waypoints; costT+= cost; timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}
	
	public void basicPRMTester(int maxIter, int maxNeighbors, double maxDist, QueryPlanner planner) {
		System.out.println(String.format("\tbasic PRMTester - %s - I=%d k=%d d=%.2f",
				planner, maxIter, maxNeighbors, maxDist));

		BasicPRM plannerPRM = new BasicPRM(iris, samplingEnv);
		plannerPRM.setMaxIter(maxIter);
		plannerPRM.setMaxNeighbors(maxNeighbors);
		plannerPRM.setMaxDistance(maxDist);
		plannerPRM.setPlanner(planner);

		Trajectory traj;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;
		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {
			t0 = System.currentTimeMillis();
			plannerPRM.setMode(QueryMode.MULTIPLE);
			traj = plannerPRM.plan(originList.get(i), destinationList.get(i), etd);
			size = Iterables.size(traj.getPositions());
			waypoints = plannerPRM.getWaypointList().size();
			cost = traj.getCost();
			time = System.currentTimeMillis() - t0;
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

	public void randomize() {
		origin = samplingEnv.sampleRandomPosition();
		destination = samplingEnv.sampleRandomPosition();
	}
}
