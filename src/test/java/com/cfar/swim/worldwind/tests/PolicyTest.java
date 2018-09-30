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
import java.time.chrono.ChronoZonedDateTime;
import java.util.Set;

import org.junit.Test;
import org.xml.sax.InputSource;

import com.binarydreamers.trees.Interval;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.render.Obstacle;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.globes.Globe;

/**
 * @author Manuel
 *
 */
public class PolicyTest {

	Iris iris;
	Globe globe;
	SamplingEnvironment samplingEnv;
	Position origin, destination;
	ZonedDateTime etd;
	
	@Test
	public void policyTester() {
		this.setScenario2();
		this.embedSWIM2();
		
		Vec4 source = globe.computePointFromPosition(origin);
		Vec4 target = globe.computePointFromPosition(destination);
		Edge edge = new Edge(origin, destination, new Line(source, target));
		edge.setCostIntervals(samplingEnv.embedIntervalTree(edge.getLine()));
		int num = edge.getCostIntervals().size();
		System.out.println("Cost Interval #"+num);
		
		source = globe.computePointFromPosition(Position.fromDegrees(48.473, -123.257, 10d));
		target = globe.computePointFromPosition(Position.fromDegrees(48.4715, -123.2525, 10d));
		// ThunderStorm to TropicalCyclone
		Edge edgeY = new Edge(origin, destination, new Line(source, target));
		edgeY.setCostIntervals(samplingEnv.embedIntervalTree(edgeY.getLine()));
		num = edgeY.getCostIntervals().size();
		for(Interval<ChronoZonedDateTime<?>> interval : edgeY.getCostIntervals()) {
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;
				System.out.println("ID: "+costInterval.getId());
			}
		}
		System.out.println("Cost Interval #"+num);
		
		// TropicalCyclone to ThunderStorm
		Edge edgeR = new Edge(origin, destination, new Line(target, source));
		edgeR.setCostIntervals(samplingEnv.embedIntervalTree(edgeR.getLine()));
		num = edgeR.getCostIntervals().size();
		for(Interval<ChronoZonedDateTime<?>> interval : edgeR.getCostIntervals()) {
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;
				System.out.println("ID: "+costInterval.getId());
			}
		}
		System.out.println("Cost Interval #"+num);
		
		double costOld=0d, costNew=0d;;
		ZonedDateTime start, end;
		
		CostPolicy costPolicy = CostPolicy.MINIMUM;
		RiskPolicy riskPolicy = RiskPolicy.SAFETY;
		
		start = etd.minusDays(2); end = etd.plusDays(1);
		costNew = edgeY.calculateCostNew(start, end, costPolicy, riskPolicy);
		costOld = edgeY.calculateCost(start, end, costPolicy, riskPolicy);
		System.out.println("Cost 0: "+costNew+"\t"+costOld);
		
		start = etd; end = etd.plusHours(1);
		costNew = edgeY.calculateCostNew(start, end, costPolicy, riskPolicy);
		costOld = edgeY.calculateCost(start, end, costPolicy, riskPolicy);
		System.out.println("Cost 1: "+costNew+"\t"+costOld);
		
		start = etd; end = etd.plusHours(3);
		costNew = edgeY.calculateCostNew(start, end, costPolicy, riskPolicy);
		costOld = edgeY.calculateCost(start, end, costPolicy, riskPolicy);
		System.out.println("Cost 2: "+costNew+"\t"+costOld);
	}
	
	public void setScenario2() {
		// Create box area in globe
		globe = new Earth();
		Sector sea = new Sector(
				Angle.fromDegrees(48.470),
				Angle.fromDegrees(48.475),
				Angle.fromDegrees(-123.26),
				Angle.fromDegrees(-123.25));
		double floor = 0d, ceilling = 50d;
		gov.nasa.worldwind.geom.Box boxNASA = Sector.computeBoundingBox(globe, 1.0, sea, floor,
				ceilling);

		// Create environment from box
		samplingEnv = new SamplingEnvironment(new Box(boxNASA));
		samplingEnv.setGlobe(globe);

		// Set planner inputs
		origin = Position.fromDegrees(48.472, -123.2599, 10d);
		destination = Position.fromDegrees(48.472, -123.2501, 40d);

		etd = ZonedDateTime.of(LocalDate.of(2018, 10, 1), LocalTime.of(0, 0), ZoneId.of("UTC"));
		iris = new Iris(origin, 1, CombatIdentification.FRIEND);
	}
	
	public void embedSWIM2() {
		this.embedObstacleS1();
		System.out.println("Obstacles total = "+samplingEnv.getObstacles().size());
		this.embedObstacleS2();
		System.out.println("Obstacles total = "+samplingEnv.getObstacles().size());
	}
	
	public void embedObstacleS1() {
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(
					new File("src/test/resources/xml/iwxxm/sigmet-victoria-tropCyc.xml"))));
			for (Obstacle obstacle : obstacles) {
				samplingEnv.embed(obstacle);
			}
			assertTrue(!samplingEnv.getObstacles().isEmpty());
		} catch (Exception e) {
			e.printStackTrace();
		}
		samplingEnv.setTime(etd);
	}
	
	public void embedObstacleS2() {
		try {
			IwxxmLoader loader = new IwxxmLoader();
			Set<Obstacle> obstacles = loader.load(new InputSource(new FileInputStream(
					new File("src/test/resources/xml/iwxxm/sigmet-victoria-thunderstrom.xml"))));
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
