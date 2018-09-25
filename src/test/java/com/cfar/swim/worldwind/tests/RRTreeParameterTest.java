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

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.Set;

import org.junit.Test;
import org.xml.sax.InputSource;

import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.ai.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.hrrt.Heuristic;
import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.airspaces.TerrainBox;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * @author Manuel
 *
 */
public class RRTreeParameterTest {
	static final int REPETITIONS = 1;
	String title;

	Iris iris;
	A320 a320;
	Globe globe;
	SamplingEnvironment samplingEnv;
	Position origin, destination;
	ZonedDateTime etd;
	
	static final double DEF_EPSILON = 15d;
	static final int DEF_BIAS = 5;
	static final int DEF_NEIGHBORS = 5;

	private enum Scenario {
		COMPLETE, SWIM, TERRAIN;
	}

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ TESTS @@@@@@@@@@@@@@@@@@@@@@@@
	 */

	/*
	@Test
	public void tuneIncrementalDist() {
		System.out.println("Tuning incremental diatnce -- repetitions = " + REPETITIONS);
	}
	*/

	@Test
	public void tuneGoalBias() {
		System.out.println("Tuning goal bias -- repetitions = " + REPETITIONS);
		Scenario[] scenarios = { Scenario.COMPLETE };
		int[] biases = { 1, 5, 10 };

		this.updateTitle("BiasTune");
		for (Scenario scenario : scenarios) {
			this.initScenario(scenario);
			// Test RRT
			for (int bias : biases)
				this.basicRRTreeTester(DEF_EPSILON, bias);
			// Test HRRT
			for (int bias : biases) {
				this.heuristicRRTreeTester(DEF_EPSILON, bias, DEF_NEIGHBORS);
			}
		}
	}

	/*
	@Test
	public void tuneNumberNeighbors() {
		System.out.println("Tuning number of neighbors -- repetitions = " + REPETITIONS);
	}
	*/

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ INIT @@@@@@@@@@@@@@@@@@@@@@@@
	 */

	/** initializes the scenario with the desired obstacles- */
	public void initScenario(Scenario scenario) {
		this.setScenario();
		switch (scenario) {
		case COMPLETE:
		default:
			this.embedSWIM();
			this.embedTerrain();
			break;
		case SWIM:
			this.embedSWIM();
			break;
		case TERRAIN:
			this.embedTerrain();
			break;
		}
		System.out.println("Initialized Scenario -->\t"+scenario.toString());
		this.printToFile("logs/" + title + ".txt", scenario.toString());
	}

	/** sets the elements of the test scenario */
	public void setScenario() {
		// Create box area in globe
		globe = new Earth();
		Sector tecnico = new Sector(
				Angle.fromDegrees(38.7381),
				Angle.fromDegrees(38.7354),
				Angle.fromDegrees(-9.1408),
				Angle.fromDegrees(-9.1364));
		double floor = 70d, ceilling = 150d;
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, tecnico, floor,
				ceilling);

		// Create environment from box
		samplingEnv = new SamplingEnvironment(new Box(boxNASA));
		samplingEnv.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(38.737, -9.137, 75);
		destination = Position.fromDegrees(38.7367, -9.1402, 105);

		etd = ZonedDateTime.of(LocalDate.of(2018, 8, 1), LocalTime.of(0, 0), ZoneId.of("UTC"));
		iris = new Iris(origin, 1, CombatIdentification.FRIEND);
		a320 = new A320(origin, 5000, CombatIdentification.FRIEND);
	}

	/** Updates the title with the given string plus the current time */
	public void updateTitle(String description) {
		String aux = LocalDate.now().toString() + String.format("_%02d%02d",
				LocalTime.now().getHour(), LocalTime.now().getMinute());
		title = aux + "_" + description;
	}

	/** Embeds a SWIM obstacle */
	public void embedSWIM() {
		this.embedObstacle1();
		this.embedObstacle2();
	}

	public void embedObstacle1() {
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(
					new File("src/test/resources/xml/iwxxm/sigmet-tecnico-tropCyc.xml"))));
			for (Obstacle obstacle : obstacles) {
				samplingEnv.embed(obstacle);
			}
			assertTrue(!samplingEnv.getObstacles().isEmpty());
		} catch (Exception e) {
			e.printStackTrace();
		}
		samplingEnv.setTime(etd);
	}

	public void embedObstacle2() {
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(
					new File("src/test/resources/xml/iwxxm/sigmet-tecnico-thunderstrom.xml"))));
			for (Obstacle obstacle : obstacles) {
				samplingEnv.embed(obstacle);
			}
			assertTrue(!samplingEnv.getObstacles().isEmpty());
		} catch (Exception e) {
			e.printStackTrace();
		}
		samplingEnv.setTime(etd);
	}

	/** Embeds a set of terrain obstacles */
	public void embedTerrain() {
		assertTrue(samplingEnv.getTerrainObstacles().isEmpty());

		File file = new File("src/test/resources/csv/TecnicoTerrain.csv");
		String line = "";
		try (BufferedReader br = new BufferedReader(new FileReader(file))) {
			while ((line = br.readLine()) != null) {
				String[] values = line.split(",");

				double lat0 = Double.parseDouble(values[0]);
				double lon0 = Double.parseDouble(values[1]);
				double lat1 = Double.parseDouble(values[2]);
				double lon1 = Double.parseDouble(values[3]);
				double left = Double.parseDouble(values[4]);
				double right = Double.parseDouble(values[5]);
				double bottom = Double.parseDouble(values[6]);
				double top = Double.parseDouble(values[7]);

				samplingEnv.embed(new TerrainBox(LatLon.fromDegrees(lat0, lon0),
						LatLon.fromDegrees(lat1, lon1), left, right, bottom, top));
			}
			assertTrue(!samplingEnv.getTerrainObstacles().isEmpty());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ LOGGERS @@@@@@@@@@@@@@@@@@@@@@@@
	 */

	/** Logger to print data of a single plan */
	public void log(double size, double waypoints, double cost, double time) {
		System.out.println(String.format("%.1f, %.1f, %.4f, %.1f", waypoints, size, cost, time));
	}

	/**
	 * Data processing of size of path, number of nodes in tree, cost of path and
	 * computation time
	 */
	public void processData(double size, double waypoints, double cost, double time) {
		size = size / REPETITIONS;
		waypoints = waypoints / REPETITIONS;
		cost = cost / REPETITIONS;
		time = time / REPETITIONS;

		System.out.println(
				String.format("Waypoints created: %.1f Path size: %.1f Cost: %.4f Time: %.1f(ms)\n",
						waypoints, size, cost, time));
		assertTrue("RRTree should find a path", size > 0);
	}

	public void printToFile(String fileID, double size, double waypoints, double cost,
			double time) {
		try (FileWriter fw = new FileWriter(fileID, true);
				BufferedWriter bw = new BufferedWriter(fw);
				PrintWriter out = new PrintWriter(bw)) {
			out.println(String.format("%.1f, %.1f, %.4f, %.1f", waypoints, size, cost, time));
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	public void printToFile(String fileID, String description) {
		try (FileWriter fw = new FileWriter(fileID, true);
				BufferedWriter bw = new BufferedWriter(fw);
				PrintWriter out = new PrintWriter(bw)) {
			out.println(String.format("%s, %s, %s, %s", description, description, description,
					description));
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ PLANNERS @@@@@@@@@@@@@@@@@@@@@@@@
	 */

	/** Basic RRT tester */
	public void basicRRTreeTester(double epsilon, int bias) {
		Strategy strategy = Strategy.EXTEND;
		RiskPolicy risk = RiskPolicy.IGNORANCE;
		int maxIter = 3000;

		System.out.println(
				String.format("\tBasic RRTreeTester - %s - e=%.1f b=%d", strategy, epsilon, bias));

		RRTreePlanner plannerRRT = new RRTreePlanner(iris, samplingEnv, epsilon, bias, maxIter,
				strategy, Extension.LINEAR);
		plannerRRT.setRiskPolicy(risk);

		Path path;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;
		// Compute plans
		this.printToFile("logs/" + title + ".txt", "RRT");
		for (int i = 0; i < REPETITIONS; i++) {
			t0 = System.currentTimeMillis();
			path = plannerRRT.plan(origin, destination, etd);
			size = Iterables.size(path.getPositions());
			waypoints = plannerRRT.getWaypointList().size();
			cost = plannerRRT.getGoal().getCost();
			time = System.currentTimeMillis() - t0;
			System.out.print(i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** Heuristic RRT tester */
	public void heuristicRRTreeTester(double epsilon, int bias, int neighbors) {
		Heuristic heuristic = Heuristic.BkRRT;
		RiskPolicy risk = RiskPolicy.IGNORANCE;
		boolean enhacements = true;
		double floor = 0;
		int maxIter = 3000;
		
		System.out.println(String.format(
				"\tHeuristic RRTreeTester - %s - e=%.1f b=%d p=%.2f n=%d (%s) -- (enhaced=%b)",
				heuristic, epsilon, bias, floor, neighbors, risk, enhacements));

		HRRTreePlanner plannerHRRT;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;

		// Compute plans
		this.printToFile("logs/" + title + ".txt", heuristic.toString());
		for (int i = 0; i < REPETITIONS; i++) {
			plannerHRRT = new HRRTreePlanner(iris, samplingEnv, epsilon, bias, maxIter,
					Strategy.EXTEND, Extension.LINEAR, floor, neighbors);
			plannerHRRT.setHeuristic(heuristic);
			plannerHRRT.setRiskPolicy(risk);
			plannerHRRT.setEnhancements(enhacements);

			t0 = System.currentTimeMillis();
			trajectory = plannerHRRT.plan(origin, destination, etd);
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				i--;
				continue;
			}
			size = Iterables.size(trajectory.getPositions());
			waypoints = plannerHRRT.getWaypointList().size();
			cost = plannerHRRT.getGoal().getCost();
			time = System.currentTimeMillis() - t0;
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

}
