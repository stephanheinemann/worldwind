/**
 * 
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

import com.cfar.swim.worldwind.ai.rrt.arrt.ARRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Sampling;
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
public class HRRTreePlannerTest {

	static final int REPETITIONS = 10;
	String title;
	
	Iris iris;
	A320 a320;
	Globe globe;
	SamplingEnvironment samplingEnv;
	Position origin, destination;
	ZonedDateTime etd;
	
	
	@Test
	public void HRRTreeTester() {
		
		this.setScenario();
		
		this.embedObstacle1();
		this.embedObstacle2();
		
		this.embedTerrain();
		
		System.out.println("Desired Repetitions #" + REPETITIONS);
		
		this.updateTitle("Enhancements");
		System.out.println("-------------\n\tSimple Heuristic\n-------------");
		this.printToFile("logs/"+title+".txt", "Simple");
		this.testerHeuristic(Heuristic.hRRT);
		System.out.println("-------------\n\tIterative Heuristic\n-------------");
		this.printToFile("logs/"+title+".txt", "Iterative");
		this.testerHeuristic(Heuristic.IkRRT);
		System.out.println("-------------\n\tBest Heuristic\n-------------");
		this.testerHeuristic(Heuristic.BkRRT);

		this.updateTitle("Comparisson");
		this.printToFile("logs/"+title+".txt", 0, 0, 0, 0); // SEPARATOR
		System.out.println("-------------\n\tBasic RRT\n-------------");
		this.printToFile("logs/"+title+".txt", "RRT_SAFETY");
		this.basicRRTreeTester(15, 5, Strategy.EXTEND, RiskPolicy.SAFETY);
		System.out.println("-------------\n\tAnytime RRT\n-------------");
		this.printToFile("logs/"+title+".txt", "ARRT_SAFETY");
		this.ARRTreeTester(15, 5, Strategy.EXTEND, RiskPolicy.SAFETY, Sampling.UNIFORM);
		
		System.out.println();
	}

	public void testerHeuristic(Heuristic heuristic) {
		// RISK POLICY OF IGNORANCE -- NO INFINITE COSTS
		System.out.println("\n\tRISK POLICY OF IGNORANCE -- NO INFINITE COSTS");
		// Original with high exploitation
		System.out.println("--Original with high exploitation");
		this.testHRRTreePlanner(15, 5, 3000, 1, 5, heuristic, false, false, RiskPolicy.IGNORANCE);
		// Modified probability with high exploitation
//		System.out.println("--Modified probability with high exploitation");
//		this.testHRRTreePlanner(15, 5, 3000, 0, 5, heuristic, true, false, RiskPolicy.IGNORANCE);
		// Modified quality with high exploitation
//		System.out.println("--Modified quality with high exploitation");
//		this.testHRRTreePlanner(15, 5, 3000, 1, 5, heuristic, false, true, RiskPolicy.IGNORANCE);
		// Fully modified with high exploitation
		System.out.println("-- Fully modified with high exploitation");
		this.testHRRTreePlanner(15, 5, 3000, 0, 5, heuristic, true, true, RiskPolicy.IGNORANCE);

		// RISK POLICY OF AVOIDANCE -- INFINITE COSTS
		System.out.println("\n\tRISK POLICY OF AVOIDANCE -- INFINITE COSTS");
		// Original with high exploitation
		System.out.println("--Original with high exploitation");
		this.testHRRTreePlanner(15, 5, 3000, 1, 5, heuristic, false, false, RiskPolicy.AVOIDANCE);
		// Modified probability with high exploitation
//		System.out.println("--Modified probability with high exploitation");
//		this.testHRRTreePlanner(15, 5, 3000, 0, 5, heuristic, true, false, RiskPolicy.AVOIDANCE);
		// Modified quality with high exploitation
//		System.out.println("--Modified quality with high exploitation");
//		this.testHRRTreePlanner(15, 5, 3000, 1, 5, heuristic, false, true, RiskPolicy.AVOIDANCE);
		// Fully modified with high exploitation
		System.out.println("-- Fully modified with high exploitation");
		this.testHRRTreePlanner(15, 5, 3000, 0, 5, heuristic, true, true, RiskPolicy.AVOIDANCE);
		
		// RISK POLICY OF IGNORANCE -- NO INFINITE COSTS
		System.out.println("\n\tRISK POLICY OF IGNORANCE -- NO INFINITE COSTS");
		// Original with high exploitation
		System.out.println("--Original with high EXPLORATION");
		this.testHRRTreePlanner(15, 5, 3000, 0.1, 5, heuristic, false, false, RiskPolicy.IGNORANCE);
		// Modified probability with high exploitation
//		System.out.println("--Modified probability with high EXPLORATION");
//		this.testHRRTreePlanner(15, 5, 3000, 1, 5, heuristic, true, false, RiskPolicy.IGNORANCE);
		// Modified quality with high exploitation
//		System.out.println("--Modified quality with high EXPLORATION");
//		this.testHRRTreePlanner(15, 5, 3000, 0.1, 5, heuristic, false, true, RiskPolicy.IGNORANCE);
		// Fully modified with high exploitation
		System.out.println("-- Fully modified with high EXPLORATION");
		this.testHRRTreePlanner(15, 5, 3000, 1, 5, heuristic, true, true, RiskPolicy.IGNORANCE);
	}
	
	
	public void testHRRTreePlanner(double epsilon, int bias, int maxIter, double floor, int neighbors, Heuristic heuristic, boolean prob, boolean qual, RiskPolicy risk) {
		System.out.println(String.format("\tHeuristic RRTreeTester - %s - e=%.1f b=%d p=%.2f n=%d (%s) -- (prob=%b | qual=%b)",
				heuristic, epsilon,	bias, floor, neighbors, risk, prob, qual));
		
		HRRTreePlanner plannerHRRT;
		
		Trajectory trajectory;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;
		
		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {
			plannerHRRT = new HRRTreePlanner(iris, samplingEnv, epsilon, bias, maxIter, Strategy.EXTEND, Extension.LINEAR, floor, neighbors);
			plannerHRRT.setHeuristic(heuristic); plannerHRRT.setRiskPolicy(risk);
			plannerHRRT.myProbability=prob; plannerHRRT.myQuality=qual;
			
			t0 = System.currentTimeMillis();
			trajectory = plannerHRRT.plan(origin, destination, etd);
			if(trajectory.isEmpty()) {
				System.out.println("No feasible solution was found");
				i--; continue;
			}
			size = Iterables.size(trajectory.getPositions());
			waypoints = plannerHRRT.getWaypointList().size();
			cost = plannerHRRT.getGoal().getCost();
			time = System.currentTimeMillis() - t0;
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/"+title+".txt", size, waypoints, cost, time);
			sizeT += size; waypointsT += waypoints; costT+= cost; timeT += time;
		}
		this.printToFile("logs/"+title+".txt", 0, 0, 0, 0); // SEPARATOR
		this.processData(sizeT, waypointsT, costT, timeT);	
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
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, tecnico, floor, ceilling);

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
	public void updateTitle(String description) {
		String aux = LocalDate.now().toString()+String.format("_%02d%02d", LocalTime.now().getHour(),LocalTime.now().getMinute());
		title = aux+"_"+description;
	}
	
	/** Embeds a SWIM obstacle */
	public void embedObstacle1() {
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(new File("src/test/resources/xml/iwxxm/sigmet-tecnico-tropCyc.xml"))));
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
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(new File("src/test/resources/xml/iwxxm/sigmet-tecnico-thunderstrom.xml"))));
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
	
	/** Logger to print data of a single plan */
	public void log(double size, double waypoints, double cost, double time) {
		System.out.println(String.format("%.1f, %.1f, %.4f, %.1f", waypoints, size, cost, time));
	}
	
	/** Data processing of size of path, number of nodes in tree, cost of path and computation time */
	public void processData(double size, double waypoints, double cost, double time) {
		size = size / REPETITIONS;
		waypoints = waypoints / REPETITIONS;
		cost = cost / REPETITIONS;
		time = time / REPETITIONS;

		System.out.println(String.format("Waypoints created: %.1f Path size: %.1f Cost: %.4f Time: %.1f(ms)\n",
				waypoints, size, cost, time));
		assertTrue("RRTree should find a path", size > 0);
	}
	
	public void printToFile(String fileID, double size, double waypoints, double cost, double time) {
		try(FileWriter fw = new FileWriter(fileID, true);
			    BufferedWriter bw = new BufferedWriter(fw);
			    PrintWriter out = new PrintWriter(bw))
			{
			    out.println(String.format("%.1f, %.1f, %.4f, %.1f", waypoints, size, cost, time));
			} catch (IOException e) {
			    e.printStackTrace();
			}
		
	}
	
	public void printToFile(String fileID, String description) {
		try(FileWriter fw = new FileWriter(fileID, true);
			    BufferedWriter bw = new BufferedWriter(fw);
			    PrintWriter out = new PrintWriter(bw))
			{
			    out.println(String.format("%s, %s, %s, %sf", description, description, description, description));
			} catch (IOException e) {
			    e.printStackTrace();
			}
		
	}
	
	public void basicRRTreeTester(double epsilon, int bias, Strategy strategy, RiskPolicy risk) {
		System.out.println(String.format("\tBasic RRTreeTester - %s - e=%.1f b=%d", strategy, epsilon, bias));

		RRTreePlanner plannerRRT = new RRTreePlanner(iris, samplingEnv, epsilon, bias, 3000, strategy, Extension.LINEAR);
		plannerRRT.setRiskPolicy(risk);

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
			System.out.print(i+"\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/"+title+".txt", size, waypoints, cost, time);
			sizeT += size; waypointsT += waypoints; costT+= cost; timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}
	
	public void ARRTreeTester(double epsilon, int bias, Strategy strategy, RiskPolicy risk, Sampling sampling) {
		System.out.println(String.format("\tBasic RRTreeTester - %s - e=%.1f b=%d", strategy, epsilon, bias));

		ARRTreePlanner plannerARRT = new ARRTreePlanner(iris, samplingEnv, epsilon, bias, 3000, strategy, Extension.LINEAR, sampling, false, 0);
		plannerARRT.setRiskPolicy(risk);
		plannerARRT.setQualityImprovement(0.01); plannerARRT.setMinimumQuality(0d); plannerARRT.setMaximumQuality(1d);

		Path path;
		double size = 0, waypoints = 0, cost = 0d, time = 0d;
		double sizeT = 0, waypointsT = 0, costT = 0d, timeT = 0d;
		long t0 = 0;
		// Compute plans
		for (int i = 0; i < REPETITIONS; i++) {
			t0 = System.currentTimeMillis();
			path = plannerARRT.plan(origin, destination, etd);
			size = Iterables.size(path.getPositions());
			waypoints = plannerARRT.getWaypointList().size();
			cost = plannerARRT.getGoal().getCost();
			time = System.currentTimeMillis() - t0;
			System.out.print(i+"\t");
			this.log(size, waypoints, cost, time);
			this.printToFile("logs/"+title+".txt", size, waypoints, cost, time);
			sizeT += size; waypointsT += waypoints; costT+= cost; timeT += time;
		}
		this.processData(sizeT, waypointsT, costT, timeT);
	}
	
}
