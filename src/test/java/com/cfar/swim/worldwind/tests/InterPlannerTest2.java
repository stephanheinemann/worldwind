/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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

import com.cfar.swim.worldwind.ai.astar.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.ai.prm.rigidprm.CollisionDelay;
import com.cfar.swim.worldwind.ai.prm.rigidprm.EnhancementMode;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryMode;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryPlanner;
import com.cfar.swim.worldwind.ai.prm.rigidprm.RigidPRM;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Sampling;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.ai.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.hrrt.Heuristic;
import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.PlanningGrid;
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

/**
 * @author Henrique Ferreira
 *
 */
public class InterPlannerTest2 {
	static final int REPETITIONS = 50;
	static final int REPETITIONS_ALG = 50;
	String title;
	String titleOrigins;

	Iris iris;
	A320 a320;
	Globe globe;
	SamplingEnvironment samplingEnv;
	PlanningGrid planningGrid;
	Position origin, destination;
	Position[] origins = new Position[REPETITIONS];
	Position[] destinations = new Position[REPETITIONS];
	ZonedDateTime etd;
	Scenario scene;

	static final double GOAL_THRESHOLD = 5d;
	static final double DEF_EPSILON = 15d;
	static final double DEF_EPSILON_SEA = 45d;
	static final int DEF_DIVISION = (int) Math.round(1000d / DEF_EPSILON_SEA);
	static final int DEF_BIAS = 5;
	static final int DEF_NEIGHBORS = 5;
	static final int DEF_MAXDISTANCE = 92;
	static final int OPTIMIZING_DISTANCE = 15;
	static final int OPTIMIZING_NEIGHBORS = 8;

	// max_distance for sea = 927,5 (0.1* 9275) longest internal diagonal
	// max distance for clutter = 92 (0.1* 920)
	// max distance for tecnico = 48,7 (0.1* 487)
	private enum Scenario {
		TECNICO, SEA, CLUTTER;
	}

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ TESTS @@@@@@@@@@@@@@@@@@@@@@@@
	 */

	@Test
	public void comparePerformance() {
		System.out.println(
				"Inter Planner Performance -- ALG repetitions = " + REPETITIONS_ALG + " repetitions = " + REPETITIONS);
		Scenario[] scenarios = {Scenario.CLUTTER, Scenario.SEA, Scenario.TECNICO };

		this.updateTitle("Cycles");
		for (Scenario scenario : scenarios) {
			this.initScenario(scenario);
			// Test RRT
			// this.heuristicRRTreeTester(DEF_EPSILON, DEF_BIAS, DEF_NEIGHBORS);
			// Test PRM
			// for (int i = 0; i < REPETITIONS_ALG; i++) {
			// System.out.println(String.format("\t \tUNIFORM Algorithm repetion #" + i));
			// this.sPRMUNIFORM(DEF_MAXDISTANCE, DEF_NEIGHBORS);
			// }
			//
			// for (int i = 0; i < REPETITIONS_ALG; i++) {
			// System.out.println(String.format("\t \tGAUSSIAN Algorithm repetion #" + i));
			// this.sPRMGAUSSIAN(DEF_MAXDISTANCE, DEF_NEIGHBORS);
			// }
			// double maxDistance = 0.5 * samplingEnv.getDiameter();
			// for (int k = 0; k < OPTIMIZING_DISTANCE; k++) {
			// double maxDistance = (0.05 + 0.01 * k) * samplingEnv.getDiameter();
			// for (int l = 0; l < OPTIMIZING_NEIGHBORS; l++) {
			// int maxNeighbors = 2 + l;
			// for (int i = 0; i < REPETITIONS_ALG; i++) {
			// System.out.println(String.format("\t \tOPTIMIZING Algorithm repetion #" +
			// i));
			// this.sPRMOptimizing(maxDistance, maxNeighbors);
			// }
			// }
			// }

			// for (int i = 0; i < REPETITIONS_ALG; i++) {
			// System.out.println(String.format("\t \tNEIGHBOR Algorithm repetion #" + i));
			// this.sPRMEnhanceNEIGHBOR(DEF_MAXDISTANCE, DEF_NEIGHBORS);
			// }
			// for (int i = 0; i < REPETITIONS_ALG; i++) {
			// System.out.println(String.format("\t \tLOCALPLANNER Algorithm repetion #" +
			// i));
			// this.sPRMEnhanceLOCALPLANNER(DEF_MAXDISTANCE, DEF_NEIGHBORS);
			// }
			for (int i = 0; i < REPETITIONS_ALG; i++) {
				System.out.println(String.format("\t \tCycles Algorithm repetion #" + i));
				this.sPRM(DEF_MAXDISTANCE, DEF_NEIGHBORS);
			}
			for (int i = 0; i < REPETITIONS_ALG; i++) {
				System.out.println(String.format("\t \tNo Cyles Algorithm repetion #" + i));
				this.sPRMsamecomponent(DEF_MAXDISTANCE, DEF_NEIGHBORS);
			}
			// // Test A*
			// this.astarTester();
		}

	}
	/*
	*/

	/*
	 * @Test public void debugger() { this.initScenario(Scenario.SEA);
	 * this.heuristicRRTreeTester(DEF_EPSILON_SEA, DEF_BIAS, 15); }
	 */

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ INIT @@@@@@@@@@@@@@@@@@@@@@@@
	 */

	/** initializes the scenario with the desired obstacles- */
	public void initScenario(Scenario scenario) {
		switch (scenario) {
		case CLUTTER:
			this.setScenarioClutter();
			this.embedTerrainClutter();
			scene = Scenario.CLUTTER;
			break;
		case SEA:
			this.setScenarioSea();
			this.embedSWIMSea();
			scene = Scenario.SEA;
			break;
		case TECNICO:
			this.setScenarioTecnico();
			this.embedSWIMTecnico();
			this.embedTerrainTecnico();
			scene = Scenario.TECNICO;
			break;
		default:
			this.setScenarioTecnico();
			this.embedSWIMTecnico();
			this.embedTerrainTecnico();
		}
		this.setOriginsDestinations();
		System.out.println("Initialized Scenario -->\t" + scenario.toString());
		// this.printToFile("logs/" + title + ".txt", scenario.toString());
	}

	/** sets the elements of the test scenario */
	public void setScenarioTecnico() {
		// Create box area in globe
		globe = new Earth();
		Sector tecnico = new Sector(Angle.fromDegrees(38.7381), Angle.fromDegrees(38.7354), Angle.fromDegrees(-9.1408),
				Angle.fromDegrees(-9.1364));
		double floor = 80d, ceilling = 109d;
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, tecnico, floor, ceilling);

		// Create environment from box
		samplingEnv = new SamplingEnvironment(new Box(boxNASA));
		samplingEnv.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(38.737, -9.137, 80);
		destination = Position.fromDegrees(38.7367, -9.1402, 105);

		etd = ZonedDateTime.of(LocalDate.of(2018, 10, 1), LocalTime.of(0, 0), ZoneId.of("UTC"));
		iris = new Iris(origin, 1, CombatIdentification.FRIEND);
		a320 = new A320(origin, 5000, CombatIdentification.FRIEND);
	}

	/** sets the elements of the CLUTTER test scenario */
	public void setScenarioClutter() {
		// Create box area in globe

		globe = new Earth();
		Sector clutter = new Sector(Angle.fromDegrees(38.71599275409678), Angle.fromDegrees(38.72068608621655),
				Angle.fromDegrees(-9.170225205236282), Angle.fromDegrees(-9.161513724287005));
		double floor = 105d, ceilling = 135d;
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, clutter, floor, ceilling);

		// Create environment from box
		samplingEnv = new SamplingEnvironment(new Box(boxNASA));
		samplingEnv.setGlobe(globe);

		// Set planner inputs
		// TODO change origin
		origin = Position.fromDegrees(38.737, -9.137, 80);
		destination = Position.fromDegrees(38.7367, -9.1402, 105);

		etd = ZonedDateTime.of(LocalDate.of(2019, 1, 10), LocalTime.of(0, 0), ZoneId.of("UTC"));
		iris = new Iris(origin, 1, CombatIdentification.FRIEND);
		a320 = new A320(origin, 5000, CombatIdentification.FRIEND);
	}

	public void setScenarioSea() {
		// Create box area in globe
		globe = new Earth();
		Sector sea = new Sector(Angle.fromDegrees(48.470), Angle.fromDegrees(48.51), Angle.fromDegrees(-123.26),
				Angle.fromDegrees(-123.15));
		double floor = 0d, ceilling = 210d;
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, sea, floor, ceilling);

		// Create environment from box
		samplingEnv = new SamplingEnvironment(new Box(boxNASA));
		samplingEnv.setGlobe(globe);

		// int division = DEF_DIVISION;
		// System.out.println("DIVISIONS PG = " + division);
		// Box envBox = new Box(boxNASA);
		// double side = envBox.getRLength() / division;
		// Cube envCube = new Cube(envBox.getOrigin(), envBox.getUnitAxes(), side);
		// int sCells = (int) Math.ceil(envBox.getSLength() / side);
		// int tCells = (int) Math.ceil(envBox.getTLength() / side);
		// planningGrid = new PlanningGrid(envCube, division, sCells, tCells);
		// planningGrid.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(48.4705, -123.259, 10d);
		destination = Position.fromDegrees(48.4745, -123.251, 40d);

		etd = ZonedDateTime.of(LocalDate.of(2019, 1, 10), LocalTime.of(0, 0), ZoneId.of("UTC"));
		iris = new Iris(origin, 1, CombatIdentification.FRIEND);
		a320 = new A320(origin, 5000, CombatIdentification.FRIEND);
	}

	public void setOriginsDestinations() {
		Position orig, dest;
		boolean swim = false, terrain = false;
		// Set origins and destinations
		for (int i = 0; i < REPETITIONS; i++) {
			do {
				orig = samplingEnv.sampleRandomPosition();
				terrain = samplingEnv.checkConflict(orig, iris);
				if (terrain)
					continue;
				swim = false;
				for (Obstacle obs : samplingEnv.getObstacles()) {
					swim = swim || samplingEnv.createBoundingBox(orig, 5).intersects(obs.getExtent(globe));
					if (swim)
						break;
				}
			} while (terrain || swim);
			do {
				dest = samplingEnv.sampleRandomPosition();
				terrain = samplingEnv.checkConflict(dest, iris);
				if (terrain)
					continue;
				swim = false;
				for (Obstacle obs : samplingEnv.getObstacles()) {
					swim = swim || samplingEnv.createBoundingBox(dest, 5).intersects(obs.getExtent(globe));
					if (swim)
						break;
				}
			} while (terrain || swim || samplingEnv.getDistance(orig, dest) < 2 * GOAL_THRESHOLD);
			origins[i] = orig;
			destinations[i] = dest;
		}
		// origins[0] = origin;
		// destinations[0] = destination;

		// Log origins and destinations
		// this.printToFile("logs/" + title + ".txt", "Origins");
		for (int i = 0; i < REPETITIONS; i++)
			this.printToFile("logs/" + titleOrigins + ".txt", origins[i].getLatitude().getDegrees() * 1000,
					origins[i].getLongitude().getDegrees() * 1000, origins[i].getAltitude(), 0d);
		for (int i = 0; i < REPETITIONS; i++)
			this.printToFile("logs/" + titleOrigins + ".txt", destinations[i].getLatitude().getDegrees() * 1000,
					destinations[i].getLongitude().getDegrees() * 1000, destinations[i].getAltitude(),
					samplingEnv.getDistance(origins[i], destinations[i]));
	}

	/** Updates the title with the given string plus the current time */
	public void updateTitle(String description) {
		String aux = LocalDate.now().toString()
				+ String.format("_%02d%02d", LocalTime.now().getHour(), LocalTime.now().getMinute());
		title = aux + "_" + description;
		titleOrigins = title + "Origins";
	}

	/** Embeds a SWIM obstacle */
	public void embedSWIMTecnico() {
		this.embedObstacle();
	}

	public void embedObstacle() {
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(
					new File("src/test/resources/xml/iwxxm/Henrique_thesisResults/sigmet-tecnico-thunderstorm.xml"))));
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
			Set<Obstacle> obstacles = loader.load(new InputSource(
					new FileInputStream(new File("src/test/resources/xml/iwxxm/sigmet-tecnico-thunderstrom.xml"))));
			for (Obstacle obstacle : obstacles) {
				samplingEnv.embed(obstacle);
			}
			assertTrue(!samplingEnv.getObstacles().isEmpty());
		} catch (Exception e) {
			e.printStackTrace();
		}
		samplingEnv.setTime(etd);
	}

	/** Embeds a set of terrain obstacles for TECNICO */
	public void embedTerrainTecnico() {
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

				samplingEnv.embed(new TerrainBox(LatLon.fromDegrees(lat0, lon0), LatLon.fromDegrees(lat1, lon1), left,
						right, bottom, top));
			}
			assertTrue(!samplingEnv.getTerrainObstacles().isEmpty());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/** Embeds a set of terrain obstacles for CLUTTER */
	public void embedTerrainClutter() {
		assertTrue(samplingEnv.getTerrainObstacles().isEmpty());

		File file = new File("src/test/resources/csv/ClutterTerrain.csv");
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

				samplingEnv.embed(new TerrainBox(LatLon.fromDegrees(lat0, lon0), LatLon.fromDegrees(lat1, lon1), left,
						right, bottom, top));
			}
			assertTrue(!samplingEnv.getTerrainObstacles().isEmpty());
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void embedSWIMSea() {
		this.embedObstacleS1();
		this.embedObstacleS2();
	}

	public void embedObstacleS1() {
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(
					new File("src/test/resources/xml/iwxxm/Henrique_thesisResults/sigmet-sea-tropCyc.xml"))));
			for (Obstacle obstacle : obstacles) {
				samplingEnv.embed(obstacle);
				// planningGrid.embed(obstacle);
			}
			assertTrue(!samplingEnv.getObstacles().isEmpty());
		} catch (Exception e) {
			e.printStackTrace();
		}
		samplingEnv.setTime(etd);
		// planningGrid.setTime(etd);
	}

	public void embedObstacleS2() {
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(
					new File("src/test/resources/xml/iwxxm/Henrique_thesisResults/sigmet-sea-thunderstorm.xml"))));
			for (Obstacle obstacle : obstacles) {
				samplingEnv.embed(obstacle);
				// planningGrid.embed(obstacle);
			}
			assertTrue(!samplingEnv.getObstacles().isEmpty());
		} catch (Exception e) {
			e.printStackTrace();
		}
		samplingEnv.setTime(etd);
		// planningGrid.setTime(etd);
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

		System.out.println(String.format("Waypoints created: %.1f Path size: %.1f Cost: %.4f Time: %.1f(ms)\n",
				waypoints, size, cost, time));
	}

	public void printToFile(String fileID, double size, double waypoints, double cost, double time) {
		try (FileWriter fw = new FileWriter(fileID, true);
				BufferedWriter bw = new BufferedWriter(fw);
				PrintWriter out = new PrintWriter(bw)) {
			out.println(String.format("%.1f; %.1f; %.4f; %.1f", waypoints, size, cost, time));
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	public void printToFile(String fileID, String description) {
		try (FileWriter fw = new FileWriter(fileID, true);
				BufferedWriter bw = new BufferedWriter(fw);
				PrintWriter out = new PrintWriter(bw)) {
			out.println(String.format("%s; %s; %s; %s", description, description, description, description));
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ PLANNERS @@@@@@@@@@@@@@@@@@@@@@@@
	 */

	/** Heuristic RRT tester */
	public void heuristicRRTreeTester(double epsilon, int bias, int neighbors) {
		Heuristic heuristic = Heuristic.BkRRT;
		RiskPolicy risk = RiskPolicy.IGNORANCE;
		boolean enhacements = true;
		double floor = 0d;
		int maxIter = 3000;

		System.out.println(String.format("\tHeuristic RRTreeTester - %s - e=%.1f b=%d p=%.2f n=%d (%s) -- (enhaced=%b)",
				heuristic, epsilon, bias, floor, neighbors, risk, enhacements));

		HRRTreePlanner plannerHRRT;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		this.printToFile("logs/" + title + ".txt", "eHRRT");
		for (int i = 0; i < REPETITIONS; i++) {
			plannerHRRT = new HRRTreePlanner(iris, samplingEnv, epsilon, bias, maxIter, Strategy.EXTEND,
					Extension.LINEAR, floor, neighbors);
			plannerHRRT.setHeuristic(heuristic);
			plannerHRRT.setRiskPolicy(risk);
			plannerHRRT.setEnhancements(enhacements);
			plannerHRRT.setGoalThreshold(GOAL_THRESHOLD);

			t0 = System.currentTimeMillis();
			trajectory = plannerHRRT.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			waypoints = plannerHRRT.getWaypointList().size();
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** original PRM tester */
	public void originalPRMTester(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		int maxIterConstruction = 500;
		int maxIterEnhancement = 150;
		boolean sameComponent = false;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NEIGHBOR;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.IGNORANCE;

		System.out.println(String.format("\toriginal PRM tester"));

		RigidPRM originalPRM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		this.printToFile("logs/" + title + ".txt", "originalPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			originalPRM = new RigidPRM(iris, samplingEnv);
			originalPRM.setCostPolicy(CostPolicy.AVERAGE);
			originalPRM.setRiskPolicy(risk);
			originalPRM.setMaxIterConstruction(maxIterConstruction);
			originalPRM.setMaxIterEnhancement(maxIterEnhancement);
			originalPRM.setSameComponent(sameComponent);
			originalPRM.setOptimalMaxDistance(optimalMaxDistance);
			originalPRM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			originalPRM.setSamplingStrategy(samplingStrategy);
			originalPRM.setEnhancement(enhancement);
			originalPRM.setDelayCollision(delayCollision);
			originalPRM.setPlanner(planner);
			originalPRM.setMode(mode);

			originalPRM.setMaxNeighbors(maxNeighbors);
			originalPRM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = originalPRM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = originalPRM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMGAUSSIAN(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 600;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.GAUSSIAN;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tGAUSSIAN tester"));

		RigidPRM prmGAUSSIAN;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmGAUSSIAN = new RigidPRM(iris, samplingEnv);
			prmGAUSSIAN.setCostPolicy(CostPolicy.AVERAGE);
			prmGAUSSIAN.setRiskPolicy(risk);
			prmGAUSSIAN.setMaxIterConstruction(maxIterConstruction);
			prmGAUSSIAN.setMaxIterEnhancement(maxIterEnhancement);
			prmGAUSSIAN.setSameComponent(sameComponent);
			prmGAUSSIAN.setOptimalMaxDistance(optimalMaxDistance);
			prmGAUSSIAN.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmGAUSSIAN.setSamplingStrategy(samplingStrategy);
			prmGAUSSIAN.setEnhancement(enhancement);
			prmGAUSSIAN.setDelayCollision(delayCollision);
			prmGAUSSIAN.setPlanner(planner);
			prmGAUSSIAN.setMode(mode);

			prmGAUSSIAN.setMaxNeighbors(maxNeighbors);
			prmGAUSSIAN.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmGAUSSIAN.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmGAUSSIAN.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMUNIFORM(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 600;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tUNIFORM tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMTester(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		int maxIterConstruction = 650;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.IGNORANCE;

		System.out.println(String.format("\tsPRM tester"));

		RigidPRM sPRM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			sPRM = new RigidPRM(iris, samplingEnv);
			sPRM.setCostPolicy(CostPolicy.AVERAGE);
			sPRM.setRiskPolicy(risk);
			sPRM.setMaxIterConstruction(maxIterConstruction);
			sPRM.setMaxIterEnhancement(maxIterEnhancement);
			sPRM.setSameComponent(sameComponent);
			sPRM.setOptimalMaxDistance(optimalMaxDistance);
			sPRM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			sPRM.setSamplingStrategy(samplingStrategy);
			sPRM.setEnhancement(enhancement);
			sPRM.setDelayCollision(delayCollision);
			sPRM.setPlanner(planner);
			sPRM.setMode(mode);

			sPRM.setMaxNeighbors(maxNeighbors);
			sPRM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = sPRM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = sPRM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** lazy PRM tester */
	public void fullLazyPRMTester(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		int maxIterConstruction = 1300;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.FULL;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.IGNORANCE;

		System.out.println(String.format("\tfull lazy sPRM tester"));

		RigidPRM lazyPRM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		this.printToFile("logs/" + title + ".txt", "fullLazysPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			lazyPRM = new RigidPRM(iris, samplingEnv);
			lazyPRM.setCostPolicy(CostPolicy.AVERAGE);
			lazyPRM.setRiskPolicy(risk);
			lazyPRM.setMaxIterConstruction(maxIterConstruction);
			lazyPRM.setMaxIterEnhancement(maxIterEnhancement);
			lazyPRM.setSameComponent(sameComponent);
			lazyPRM.setOptimalMaxDistance(optimalMaxDistance);
			lazyPRM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			lazyPRM.setSamplingStrategy(samplingStrategy);
			lazyPRM.setEnhancement(enhancement);
			lazyPRM.setDelayCollision(delayCollision);
			lazyPRM.setPlanner(planner);
			lazyPRM.setMode(mode);

			lazyPRM.setMaxNeighbors(maxNeighbors);
			lazyPRM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = lazyPRM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = lazyPRM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** lazy PRM tester */
	public void halfLazyPRMTester(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		int maxIterConstruction = 950;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.HALF;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.IGNORANCE;

		System.out.println(String.format("\thalf Lazy sPRM tester"));

		RigidPRM lazyPRM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		this.printToFile("logs/" + title + ".txt", "halfLazysPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			lazyPRM = new RigidPRM(iris, samplingEnv);
			lazyPRM.setCostPolicy(CostPolicy.AVERAGE);
			lazyPRM.setRiskPolicy(risk);
			lazyPRM.setMaxIterConstruction(maxIterConstruction);
			lazyPRM.setMaxIterEnhancement(maxIterEnhancement);
			lazyPRM.setSameComponent(sameComponent);
			lazyPRM.setOptimalMaxDistance(optimalMaxDistance);
			lazyPRM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			lazyPRM.setSamplingStrategy(samplingStrategy);
			lazyPRM.setEnhancement(enhancement);
			lazyPRM.setDelayCollision(delayCollision);
			lazyPRM.setPlanner(planner);
			lazyPRM.setMode(mode);

			lazyPRM.setMaxNeighbors(maxNeighbors);
			lazyPRM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = lazyPRM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = lazyPRM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** PRM star tester */
	public void PRMStarTester(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		int maxIterConstruction = 275;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = true;
		boolean optimalMaxDistance = true;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.IGNORANCE;

		System.out.println(String.format("\tPRM Star tester"));

		RigidPRM prmStar;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		this.printToFile("logs/" + title + ".txt", "PRMStar");
		for (int i = 0; i < REPETITIONS; i++) {
			prmStar = new RigidPRM(iris, samplingEnv);
			prmStar.setCostPolicy(CostPolicy.AVERAGE);
			prmStar.setRiskPolicy(risk);
			prmStar.setMaxIterConstruction(maxIterConstruction);
			prmStar.setMaxIterEnhancement(maxIterEnhancement);
			prmStar.setSameComponent(sameComponent);
			prmStar.setMaxNeighbors(maxNeighbors);
			prmStar.setMaxDistance(maxDistance);
			prmStar.setOptimalMaxDistance(optimalMaxDistance);
			prmStar.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmStar.setSamplingStrategy(samplingStrategy);
			prmStar.setEnhancement(enhancement);
			prmStar.setDelayCollision(delayCollision);
			prmStar.setPlanner(planner);
			prmStar.setMode(mode);

			t0 = System.currentTimeMillis();
			trajectory = prmStar.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmStar.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** basic PRM tester */
	public void astarTester() {
		RiskPolicy risk = RiskPolicy.IGNORANCE;
		CostPolicy costPolicy = CostPolicy.AVERAGE;

		System.out.println(String.format("\tforward A* tester"));

		ForwardAStarPlanner plannerAS;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		this.printToFile("logs/" + title + ".txt", "A*");
		for (int i = 0; i < REPETITIONS; i++) {
			plannerAS = new ForwardAStarPlanner(iris, planningGrid);
			plannerAS.setCostPolicy(costPolicy);
			plannerAS.setRiskPolicy(risk);

			t0 = System.currentTimeMillis();
			trajectory = plannerAS.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
				time = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = 0;
				cost = planningGrid.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMNONE(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 600;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tNONE tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMHALF(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 0;
		// depend on scenario
		switch (scene) {
		case CLUTTER:
			maxIterConstruction = 750;
			break;
		case SEA:
			maxIterConstruction = 600;
			break;
		case TECNICO:
			maxIterConstruction = 1200;
			break;
		default:
		}

		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.HALF;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tHALF tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMFULL(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 0;
		// depend on scenario
		switch (scene) {
		case CLUTTER:
			maxIterConstruction = 1200;
			break;
		case SEA:
			maxIterConstruction = 600;
			break;
		case TECNICO:
			maxIterConstruction = 2500;
			break;
		default:
		}
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.FULL;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tFULL tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMEnhanceNONE(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 600;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tNONE tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMEnhanceNEIGHBOR(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 400;
		int maxIterEnhancement = 200;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NEIGHBOR;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tNEIGHBOR tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMEnhanceLOCALPLANNER(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 400;
		int maxIterEnhancement = 200;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.LOCALPLANNER;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tLOCALPLANNER tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRM(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 600;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;
		System.out.println(String.format("\tCycles tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMsamecomponent(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		maxDistance = samplingEnv.getDiameter() * 0.1;
		int maxIterConstruction = 600;
		int maxIterEnhancement = 0;
		boolean sameComponent = false;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.IGNORANCE;
		System.out.println(String.format("\tNo cycles tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/" + title + ".txt", size, waypoints, cost, time);
			sizeT += size;
			waypointsT += waypoints;
			costT += cost;
			timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}

	/** simplified PRM tester */
	public void sPRMOptimizing(double maxDistance, int maxNeighbors) {
		samplingEnv.getWaypointList().clear();
		samplingEnv.getEdgeList().clear();

		int maxIterConstruction = 600;
		int maxIterEnhancement = 0;
		boolean sameComponent = true;
		// int maxNeighbors = 15;
		// double maxDistance = 50;
		boolean optimalMaxNeighbors = false;
		boolean optimalMaxDistance = false;
		Sampling samplingStrategy = Sampling.UNIFORM;
		EnhancementMode enhancement = EnhancementMode.NONE;
		CollisionDelay delayCollision = CollisionDelay.NONE;
		QueryPlanner planner = QueryPlanner.FAS;
		QueryMode mode = QueryMode.MULTIPLE;

		RiskPolicy risk = RiskPolicy.SAFETY;

		System.out.println(String.format("\tOptimizing tester"));

		RigidPRM prmUNIFORM;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0, t1 = 0;

		// Compute plans
		// this.printToFile("logs/" + title + ".txt", "sPRM");
		for (int i = 0; i < REPETITIONS; i++) {
			prmUNIFORM = new RigidPRM(iris, samplingEnv);
			prmUNIFORM.setCostPolicy(CostPolicy.AVERAGE);
			prmUNIFORM.setRiskPolicy(risk);
			prmUNIFORM.setMaxIterConstruction(maxIterConstruction);
			prmUNIFORM.setMaxIterEnhancement(maxIterEnhancement);
			prmUNIFORM.setSameComponent(sameComponent);
			prmUNIFORM.setOptimalMaxDistance(optimalMaxDistance);
			prmUNIFORM.setOptimalMaxNeighbors(optimalMaxNeighbors);
			prmUNIFORM.setSamplingStrategy(samplingStrategy);
			prmUNIFORM.setEnhancement(enhancement);
			prmUNIFORM.setDelayCollision(delayCollision);
			prmUNIFORM.setPlanner(planner);
			prmUNIFORM.setMode(mode);

			prmUNIFORM.setMaxNeighbors(maxNeighbors);
			prmUNIFORM.setMaxDistance(maxDistance);

			t0 = System.currentTimeMillis();
			trajectory = prmUNIFORM.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
			} else {
				size = Iterables.size(trajectory.getPositions());
				waypoints = prmUNIFORM.getWaypointList().size();
				cost = samplingEnv.getDistance(trajectory.getCost());
			}
			time = t1 - t0;
			System.out.print("Iter  #" + i + "\t");
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
