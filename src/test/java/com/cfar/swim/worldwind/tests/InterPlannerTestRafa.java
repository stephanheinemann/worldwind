/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
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

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDate;
import java.time.LocalTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import org.junit.Test;
import com.cfar.swim.worldwind.planners.cgs.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.planners.rl.dqn.DQNPlanner;
import com.cfar.swim.worldwind.planners.rl.priordqn.PriorDQNPlanner;
import com.cfar.swim.worldwind.planners.rrt.Extension;
import com.cfar.swim.worldwind.planners.rrt.Strategy;
import com.cfar.swim.worldwind.planners.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.environments.RLEnvironment;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.airspaces.ObstacleBox;
import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;

/** 
 * Performs planner tests.
 * 
 * @author Rafaela Seguro
 *
 */
public class InterPlannerTestRafa {
	static final int REPETITIONS = 100;
	String title;

	Iris iris;
	A320 a320;
	Globe globe;
	PlanningContinuum planningContinuum;
	PlanningGrid planningGrid;
	RLEnvironment rlEnvironment;
	Position origin, destination;
	Position[] origins = new Position[REPETITIONS];
	Position[] destinations = new Position[REPETITIONS];
	ZonedDateTime etd;
	
	RiskPolicy risk = RiskPolicy.SAFETY;
	CostPolicy costPolicy = CostPolicy.AVERAGE; 

	static final double GOAL_THRESHOLD = 5d;
	static final double DEF_EPSILON = 15d;
	static final int DEF_DIVISION = 50;
	static final int DEF_BIAS = 5;
	static final int DEF_NEIGHBORS = 15;


	private enum Scenario {
		EMPTY, OCEAN, CAMP;
	}

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ TESTS @@@@@@@@@@@@@@@@@@@@@@@@
	 */


	@Test
	public void comparePerformance() {
		Scenario[] scenarios = { Scenario.EMPTY, Scenario.OCEAN, Scenario.CAMP };

		PrintWriter outputFile = null;
		
		
		for (Scenario scenario : scenarios) {
			this.initScenario(scenario);
			
			// Test DQN
			String name = "DQN_" + scenario.toString();
			try { outputFile = new PrintWriter(name); } catch (FileNotFoundException e) { e.printStackTrace();}
			outputFile.println("Iteration, Success, Size, Cost, Waypoints, Time");
			
			this.dqnTester(outputFile); 
			
			outputFile.close();
			
			// Test PriorDQN
			name = "PriorDQN_" + scenario.toString();
			try { outputFile = new PrintWriter(name); } catch (FileNotFoundException e) { e.printStackTrace();}
			outputFile.println("Iteration, Success, Size, Cost, Waypoints, Time");
			
			this.priordqnTester(outputFile);
			
			outputFile.close();
			
			// Test hRRT
			name = "hRRT_" + scenario.toString();
			try { outputFile = new PrintWriter(name); } catch (FileNotFoundException e) { e.printStackTrace();}
			outputFile.println("Epsilon, " + DEF_EPSILON + ", Bias, " + DEF_BIAS + ", Neighbors, " + DEF_NEIGHBORS + ", Goal Threshold, " + GOAL_THRESHOLD);
			outputFile.println("Iteration, Success, Size, Cost, Waypoints, Time");
			
			this.heuristicRRTreeTester(DEF_EPSILON, DEF_BIAS, DEF_NEIGHBORS, outputFile);
			
			outputFile.close();
			
			// Test A*
			name = "AStar_" + scenario.toString();
			try { outputFile = new PrintWriter(name); } catch (FileNotFoundException e) { e.printStackTrace();}
			outputFile.println("Division, " + DEF_DIVISION);
			outputFile.println("Iteration, Success, Size, Cost, Waypoints, Time");
			
			this.astarTester(outputFile);
			
			outputFile.close();
		}

	}
	/*
	*/

	/*
	@Test
	public void debugger() {
		this.initScenario(Scenario.SEA);
		this.heuristicRRTreeTester(DEF_EPSILON_SEA, DEF_BIAS, 15);
	}
	*/

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ INIT @@@@@@@@@@@@@@@@@@@@@@@@
	 */

	/** initializes the scenario with the desired obstacles- */
	public void initScenario(Scenario scenario) {
		switch (scenario) {
		case OCEAN:
			this.setScenario1();
			this.embedObstacles1();
			break;
		case CAMP:
			this.setScenario2();
			this.embedObstacles2();
			break;
		case EMPTY:
		default:
			this.setScenario2();
		}
		
		this.setOrignisDestinations(scenario);
		System.out.println("Initialized Scenario -->\t" + scenario.toString());
	}

	/** sets the elements of the test scenario 1 */
	public void setScenario1() {
		// Create box area in globe
		globe = new Earth();
		Sector ocean = new Sector(
				Angle.fromDegrees(48.470),
				Angle.fromDegrees(48.475),
				Angle.fromDegrees(-123.26),
				Angle.fromDegrees(-123.25));
		double floor = 0d, ceilling = 200d;
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, ocean, floor,
				ceilling);

		// Create RL Environment
		rlEnvironment = new RLEnvironment(new Box(boxNASA));
		rlEnvironment.setGlobe(globe); 
		
		// Create planning continuum
		planningContinuum = new PlanningContinuum(new Box(boxNASA));
		planningContinuum.setGlobe(globe);
		
		// Create planning grid
		int division = DEF_DIVISION;
		Box envBox = new Box(boxNASA);
		double side = envBox.getRLength() / division;
		Cube envCube = new Cube(envBox.getOrigin(), envBox.getUnitAxes(), side);
		int sCells = (int) Math.ceil(envBox.getSLength() / side);
		int tCells = (int) Math.ceil(envBox.getTLength() / side);
		planningGrid = new PlanningGrid(envCube, division, sCells, tCells);
		planningGrid.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(48.4705, -123.259, 10d);
		destination = Position.fromDegrees(48.4745, -123.251, 40d);

		etd = ZonedDateTime.of(LocalDate.of(2023, 12, 6), LocalTime.of(18, 1), ZoneId.of("UTC"));
		iris = new Iris(origin, 1, CombatIdentification.FRIEND);
	}

	public void setScenario2() {
		// Create box area in globe
		globe = new Earth();
		Sector camp = new Sector(
				Angle.fromDegrees(48.55447370425059),
				Angle.fromDegrees(48.558105277138175),
				Angle.fromDegrees(-123.38587626720192),
				Angle.fromDegrees(-123.37827827273274));
		double floor = 0d, ceilling = 300d;
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, camp, floor,
				ceilling);

		// Create RL Environment
		rlEnvironment = new RLEnvironment(new Box(boxNASA));
		rlEnvironment.setGlobe(globe); 
		
		// Create planning continuum
		planningContinuum = new PlanningContinuum(new Box(boxNASA));
		planningContinuum.setGlobe(globe);
		
		// Create planning grid
		int division = DEF_DIVISION;
		Box envBox = new Box(boxNASA);
		double side = envBox.getRLength() / division;
		Cube envCube = new Cube(envBox.getOrigin(), envBox.getUnitAxes(), side);
		int sCells = (int) Math.ceil(envBox.getSLength() / side);
		int tCells = (int) Math.ceil(envBox.getTLength() / side);
		planningGrid = new PlanningGrid(envCube, division, sCells, tCells);
		planningGrid.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(48.5547, -123.3850, 20d);
		destination = Position.fromDegrees(48.5578, -123.3787, 200d);

		etd = ZonedDateTime.of(LocalDate.of(2023, 12, 6), LocalTime.of(18, 1), ZoneId.of("UTC"));
		iris = new Iris(origin, 1, CombatIdentification.FRIEND);
	}

	public void setOrignisDestinations(Scenario scenario) {
		Position orig, dest;
		boolean obstacle = false;
		// Set origins and destinations
		for (int i = 0; i < REPETITIONS; i++) {
			do {
				orig = planningContinuum.sampleRandomUniformPosition();
				ObstacleSphere origObstacle = new ObstacleSphere(orig, 5);
				obstacle = false;
				for(Obstacle obs : planningContinuum.getObstacles()) {
					obstacle = obstacle || obs.intersects(globe, origObstacle);
					if(obstacle)
						break;
				}
			} while (obstacle);
			do {
				dest = planningContinuum.sampleRandomUniformPosition();
				ObstacleSphere destObstacle = new ObstacleSphere(dest, 5);
				obstacle = false;
				for(Obstacle obs : planningContinuum.getObstacles()) {
					obstacle = obstacle || obs.intersects(globe, destObstacle);
					if(obstacle)
						break;
				}
			} while (obstacle || planningContinuum.getDistance(orig, dest)<2*GOAL_THRESHOLD);
			
			origins[i]=orig;
			destinations[i]=dest;
		}
		origins[0] = origin;
		destinations[0] = destination;

		// Log origins and destinations to file
		PrintWriter outputFile = null;
		String name = "Origins and Destinations (" + scenario.toString() + ")";
		try { outputFile = new PrintWriter(name); } catch (FileNotFoundException e) { e.printStackTrace(); }
		
		outputFile.println("Number, Latitude, Longitude, Altitude, Latitude, Longitude, Altitude, Distance");
		
		for (int i = 0; i < REPETITIONS; i++) {
		
			outputFile.println(i + ", " + origins[i].getLatitude().getDegrees() + ", " + origins[i].getLongitude().getDegrees() + ", " +
			origins[i].getAltitude() + ", " + destinations[i].getLatitude().getDegrees() + ", " + destinations[i].getLongitude().getDegrees() + ", " +
			destinations[i].getAltitude() + ", " + planningContinuum.getDistance(origins[i], destinations[i]));
		}
		
		outputFile.close();
		
	}

	/** Updates the title with the given string plus the current time */
	public void updateTitle(String description) {
		String aux = LocalDate.now().toString() + String.format("_%02d%02d",
				LocalTime.now().getHour(), LocalTime.now().getMinute());
		title = aux + "_" + description;
	}


	public void embedObstacles1() {
		
		ZonedDateTime t1 = ZonedDateTime.of(LocalDate.of(2023, 12, 5), LocalTime.of(18, 1), ZoneId.of("UTC"));;
		ZonedDateTime t2 = ZonedDateTime.of(LocalDate.of(2023, 12, 7), LocalTime.of(18, 1), ZoneId.of("UTC"));
		
		// Create obstacles
		Position p1 = new Position(Angle.fromDegrees(48.4737), Angle.fromDegrees(-123.2574), 120);
		ObstacleSphere o1 = new ObstacleSphere(p1, 50);
		o1.setCostInterval(new CostInterval("CFAR001", t1, t2, 80));
		
		LatLon p2 = new LatLon(Angle.fromDegrees(48.4716), Angle.fromDegrees(-123.2528));
		ObstacleCylinder o2 = new ObstacleCylinder(p2, 0, 200, 130);
		o2.setCostInterval(new CostInterval("airspace1", t1, t2, 40));
		
		// Embeds obstacles
		rlEnvironment.embed(o1);
		planningContinuum.embed(o1);
		planningGrid.embed(o1);
		rlEnvironment.embed(o2);
		planningContinuum.embed(o2);
		planningGrid.embed(o2);
 
	}

	public void embedObstacles2() {
		
		ZonedDateTime t1 = ZonedDateTime.of(LocalDate.of(2023, 12, 5), LocalTime.of(18, 1), ZoneId.of("UTC"));;
		ZonedDateTime t2 = ZonedDateTime.of(LocalDate.of(2023, 12, 7), LocalTime.of(18, 1), ZoneId.of("UTC"));
		
		// Create obstacles
		LatLon l1 = new LatLon(Angle.fromDegrees(48.5569), Angle.fromDegrees(-123.3849));
		LatLon l2 = new LatLon(Angle.fromDegrees(48.55753), Angle.fromDegrees(-123.3833));
		ObstacleBox o1 = new ObstacleBox(l1, l2, 50, 50, 50, 100);
		o1.setCostInterval(new CostInterval("airspace1", t1, t2, 70));
		
		Position p1 = new Position(Angle.fromDegrees(48.5556), Angle.fromDegrees(-123.3839), 120);
		ObstacleSphere o2 = new ObstacleSphere(p1, 40);
		o2.setCostInterval(new CostInterval("CFAR001", t1, t2, 100));
		
		LatLon l3 = new LatLon(Angle.fromDegrees(48.5553), Angle.fromDegrees(-123.3813));
		LatLon l4 = new LatLon(Angle.fromDegrees(48.5553), Angle.fromDegrees(-123.3794));
		ObstacleBox o3 = new ObstacleBox(l3, l4, 50, 50, 140, 230);
		o3.setCostInterval(new CostInterval("airspace2", t1, t2, 40));
		
		LatLon p2 = new LatLon(Angle.fromDegrees(48.557), Angle.fromDegrees(-123.3803));
		ObstacleCylinder o4 = new ObstacleCylinder(p2, 0, 300, 80);
		o4.setCostInterval(new CostInterval("airspace3", t1, t2, 20));
		
		// Embeds obstacles
		rlEnvironment.embed(o1);
		planningContinuum.embed(o1);
		planningGrid.embed(o1);
		rlEnvironment.embed(o2);
		planningContinuum.embed(o2);
		planningGrid.embed(o2);
		rlEnvironment.embed(o3);
		planningContinuum.embed(o3);
		planningGrid.embed(o3);
		rlEnvironment.embed(o4);
		planningContinuum.embed(o4);
		planningGrid.embed(o4);

	}

//	/*
//	 * @@@@@@@@@@@@@@@@@@@@@@@@ LOGGERS @@@@@@@@@@@@@@@@@@@@@@@@
//	 */
//
//	/** Logger to print data of a single plan */
//	public void log(double size, double waypoints, double cost, double time) {
//		System.out.println(String.format("%.1f, %.1f, %.4f, %.1f", waypoints, size, cost, time));
//	}
//
//	/**
//	 * Data processing of size of path, number of nodes in tree, cost of path and computation time
//	 */
//	public void processData(double size, double waypoints, double cost, double time) {
//		size = size / REPETITIONS;
//		waypoints = waypoints / REPETITIONS;
//		cost = cost / REPETITIONS;
//		time = time / REPETITIONS;
//
//		System.out.println(
//				String.format("Waypoints created: %.1f Path size: %.1f Cost: %.4f Time: %.1f(ms)\n",
//						waypoints, size, cost, time));
//	}
//
//	public void printToFile(String fileID, double size, double waypoints, double cost,
//			double time) {
//		try (FileWriter fw = new FileWriter(fileID, true);
//				BufferedWriter bw = new BufferedWriter(fw);
//				PrintWriter out = new PrintWriter(bw)) {
//			out.println(String.format("%.1f; %.1f; %.4f; %.1f", waypoints, size, cost, time));
//		} catch (IOException e) {
//			e.printStackTrace();
//		}
//
//	}
//
//	public void printToFile(String fileID, String description) {
//		try (FileWriter fw = new FileWriter(fileID, true);
//				BufferedWriter bw = new BufferedWriter(fw);
//				PrintWriter out = new PrintWriter(bw)) {
//			out.println(String.format("%s; %s; %s; %s", description, description, description,
//					description));
//		} catch (IOException e) {
//			e.printStackTrace();
//		}
//
//	}

	/*
	 * @@@@@@@@@@@@@@@@@@@@@@@@ PLANNERS @@@@@@@@@@@@@@@@@@@@@@@@
	 */
	
	/** A* tester */
	public void dqnTester(PrintWriter outputFile) {

		System.out.println("DQN tester");

		DQNPlanner plannerDQN;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		boolean success;
		long t0 = 0, t1 = 0;
		
		// Create planner (to train only once)
		plannerDQN = new DQNPlanner(iris, rlEnvironment);
		plannerDQN.setCostPolicy(costPolicy);
		plannerDQN.setRiskPolicy(risk);
		
		System.out.println("Training complete");

		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {

			t0 = System.currentTimeMillis();
			trajectory = plannerDQN.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
				time = 0;
				success = false;
			} else {
				size = trajectory.getLength(globe);
				cost = trajectory.getCost();
				waypoints = trajectory.getWaypointsLength();
				time = t1 - t0;
				success = true;
			}
			
			System.out.println(i + ", " + success + ", " + size + ", " + cost + ", " + waypoints + ", " + time);
			outputFile.println(i + ", " + success + ", " + size + ", " + cost + ", " + waypoints + ", " + time);
		}
	}
	
	/** A* tester */
	public void priordqnTester(PrintWriter outputFile) {

		System.out.println("PriorDQN tester");

		PriorDQNPlanner plannerPriorDQN;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		boolean success;
		long t0 = 0, t1 = 0;
		
		// Create planner (to train only once)
		plannerPriorDQN = new PriorDQNPlanner(iris, rlEnvironment);
		plannerPriorDQN.setCostPolicy(costPolicy);
		plannerPriorDQN.setRiskPolicy(risk);
		
		System.out.println("Training complete");

		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {

			t0 = System.currentTimeMillis();
			trajectory = plannerPriorDQN.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
				time = 0;
				success = false;
			} else {
				size = trajectory.getLength(globe);
				cost = trajectory.getCost();
				waypoints = trajectory.getWaypointsLength();
				time = t1 - t0;
				success = true;
			}
			
			System.out.println(i + ", " + success + ", " + size + ", " + cost + ", " + waypoints + ", " + time);
			outputFile.println(i + ", " + success + ", " + size + ", " + cost + ", " + waypoints + ", " + time);
		}
	}

	/** Heuristic RRT tester */
	public void heuristicRRTreeTester(double epsilon, int bias, int neighbors, PrintWriter outputFile) {
		
		int maxIter = 3000;

		System.out.println("hRRT Tester");

		HRRTreePlanner plannerHRRT;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		boolean success;
		long t0 = 0, t1 =0;
		
		plannerHRRT = new HRRTreePlanner(iris, planningContinuum);
		plannerHRRT.setRiskPolicy(risk);
		plannerHRRT.setCostPolicy(costPolicy);
		plannerHRRT.setGoalThreshold(GOAL_THRESHOLD);
		plannerHRRT.setEpsilon(epsilon);
		plannerHRRT.setBias(bias);
		plannerHRRT.setMaxIterations(maxIter);
		plannerHRRT.setNeighborLimit(neighbors);

		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {

			t0 = System.currentTimeMillis();
			trajectory = plannerHRRT.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				cost = 0;
				waypoints = 0;
				time = 0;
				success = false;
			} else {
				size = trajectory.getLength(globe);
				cost = trajectory.getCost();
				waypoints = trajectory.getWaypointsLength();
				time = t1 - t0;
				success = true;
			}
			
			System.out.println(i + ", " + success + ", " + size + ", " + cost + ", " + waypoints + ", " + time);
			outputFile.println(i + ", " + success + ", " + size + ", " + cost + ", " + waypoints + ", " + time);
		}
	}



	/** A* tester */
	public void astarTester(PrintWriter outputFile) {

		System.out.println("A* Tester");

		ForwardAStarPlanner plannerAS;

		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		boolean success;
		long t0 = 0, t1 = 0;
		
		plannerAS = new ForwardAStarPlanner(iris, planningGrid);
		plannerAS.setCostPolicy(costPolicy);
		plannerAS.setRiskPolicy(risk);

		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {

			t0 = System.currentTimeMillis();
			trajectory = plannerAS.plan(origins[i], destinations[i], etd);
			t1 = System.currentTimeMillis();
			if (trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				size = 0;
				waypoints = 0;
				cost = 0;
				time = 0;
				success = false;
			} else {
				size = trajectory.getLength(globe);
				cost = trajectory.getCost();
				waypoints = trajectory.getWaypointsLength();
				time = t1 - t0;
				success = true;
			}
			
			System.out.println(i + ", " + success + ", " + size + ", " + cost + ", " + waypoints + ", " + time);
			outputFile.println(i + ", " + success + ", " + size + ", " + cost + ", " + waypoints + ", " + time);
		}
	}

}