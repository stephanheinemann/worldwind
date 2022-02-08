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

import static org.junit.Assert.assertEquals;

import java.time.ZoneId;
import java.time.ZonedDateTime;

import org.junit.Test;

import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.precision.Precision;
import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;

/**
 * Performs operational cost and risk policies tests.
 * 
 * @author Stephan Heineamnn
 *
 */
public class PoliciesTest {
	
	/**
	 * Tests cost policies for cost intervals.
	 */
	@Test
	public void testCostPolicy() {
		Box box = new Box(new Vec4(10d));
		PlanningContinuum continuum = new PlanningContinuum(box);
		continuum.setGlobe(new Earth());
		
		// edge with 10 hour ETE
		Waypoint first = new Waypoint(continuum.getGlobe()
				.computePositionFromPoint(box.getBottomCenter()));
		ZonedDateTime firstEto = ZonedDateTime.of(2021, 1, 1, 0, 0, 0, 0, ZoneId.of("UTC"));
		first.setEto(firstEto);
		Waypoint second = new Waypoint(continuum.getGlobe()
				.computePositionFromPoint(box.getTopCenter()));
		ZonedDateTime secondEto = ZonedDateTime.of(2021, 1, 1, 10, 0, 0, 0, ZoneId.of("UTC"));
		second.setEto(secondEto);
		Edge edge = new Edge(continuum, first, second);
		
		// base costs only
		double minCost = edge.getCost(firstEto, secondEto, CostPolicy.MINIMUM, RiskPolicy.IGNORANCE);
		double avgCost = edge.getCost(firstEto, secondEto, CostPolicy.AVERAGE, RiskPolicy.IGNORANCE);
		double maxCost = edge.getCost(firstEto, secondEto, CostPolicy.MAXIMUM, RiskPolicy.IGNORANCE);
		assertEquals(continuum.getBaseCost(), minCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost(), avgCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost(), maxCost, PrecisionDouble.UNIT_DECA_MICRO);
		
		// enclosing costs
		CostInterval matchingInterval = new CostInterval("matching", firstEto, secondEto, 10d);
		edge.addCostInterval(matchingInterval);
		minCost = edge.getCost(firstEto, secondEto, CostPolicy.MINIMUM, RiskPolicy.IGNORANCE);
		avgCost = edge.getCost(firstEto, secondEto, CostPolicy.AVERAGE, RiskPolicy.IGNORANCE);
		maxCost = edge.getCost(firstEto, secondEto, CostPolicy.MAXIMUM, RiskPolicy.IGNORANCE);
		assertEquals(continuum.getBaseCost() + 10d, minCost, Precision.UNIT_HECTO_MICRO);
		assertEquals(continuum.getBaseCost() + 10d, avgCost, Precision.UNIT_HECTO_MICRO);
		assertEquals(continuum.getBaseCost() + 10d, maxCost, Precision.UNIT_HECTO_MICRO);
		
		CostInterval enclosingInterval = new CostInterval("enclosing",
				firstEto.minusHours(1l), secondEto.plusHours(1l), 10d);
		edge.addCostInterval(enclosingInterval);
		minCost = edge.getCost(firstEto, secondEto, CostPolicy.MINIMUM, RiskPolicy.IGNORANCE);
		avgCost = edge.getCost(firstEto, secondEto, CostPolicy.AVERAGE, RiskPolicy.IGNORANCE);
		maxCost = edge.getCost(firstEto, secondEto, CostPolicy.MAXIMUM, RiskPolicy.IGNORANCE);
		assertEquals(continuum.getBaseCost() + 20d, minCost, Precision.UNIT_HECTO_MICRO);
		assertEquals(continuum.getBaseCost() + 20d, avgCost, Precision.UNIT_HECTO_MICRO);
		assertEquals(continuum.getBaseCost() + 20d, maxCost, Precision.UNIT_HECTO_MICRO);
		
		// overlapping costs
		CostInterval overlappingInterval1 = new CostInterval("overlapping1",
				firstEto.minusHours(1l), secondEto.minusHours(5l), 10d);
		edge.addCostInterval(overlappingInterval1);
		minCost = edge.getCost(firstEto, secondEto, CostPolicy.MINIMUM, RiskPolicy.IGNORANCE);
		avgCost = edge.getCost(firstEto, secondEto, CostPolicy.AVERAGE, RiskPolicy.IGNORANCE);
		maxCost = edge.getCost(firstEto, secondEto, CostPolicy.MAXIMUM, RiskPolicy.IGNORANCE);
		assertEquals(continuum.getBaseCost() + 20d, minCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost() + 25d, avgCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost() + 30d, maxCost, PrecisionDouble.UNIT_DECA_MICRO);
		
		CostInterval overlappingInterval2 = new CostInterval("overlapping2",
				firstEto.plusHours(5l), secondEto.plusHours(1l), 10d);
		edge.addCostInterval(overlappingInterval2);
		minCost = edge.getCost(firstEto, secondEto, CostPolicy.MINIMUM, RiskPolicy.IGNORANCE);
		avgCost = edge.getCost(firstEto, secondEto, CostPolicy.AVERAGE, RiskPolicy.IGNORANCE);
		maxCost = edge.getCost(firstEto, secondEto, CostPolicy.MAXIMUM, RiskPolicy.IGNORANCE);
		assertEquals(continuum.getBaseCost() + 30d, minCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost() + 30d, avgCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost() + 30d, maxCost, PrecisionDouble.UNIT_DECA_MICRO);
		
		CostInterval overlappingInterval3 = new CostInterval("overlapping3",
				firstEto.plusMinutes(150l), secondEto.minusMinutes(150l), 10d);
		edge.addCostInterval(overlappingInterval3);
		minCost = edge.getCost(firstEto, secondEto, CostPolicy.MINIMUM, RiskPolicy.IGNORANCE);
		avgCost = edge.getCost(firstEto, secondEto, CostPolicy.AVERAGE, RiskPolicy.IGNORANCE);
		maxCost = edge.getCost(firstEto, secondEto, CostPolicy.MAXIMUM, RiskPolicy.IGNORANCE);
		assertEquals(continuum.getBaseCost() + 30d, minCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost() + 35d, avgCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost() + 40d, maxCost, PrecisionDouble.UNIT_DECA_MICRO);
		
		CostInterval overlappingInterval4 = new CostInterval("overlapping4",
				firstEto.plusHours(1l), secondEto.minusHours(5l), 10d);
		edge.addCostInterval(overlappingInterval4);
		minCost = edge.getCost(firstEto, secondEto, CostPolicy.MINIMUM, RiskPolicy.IGNORANCE);
		avgCost = edge.getCost(firstEto, secondEto, CostPolicy.AVERAGE, RiskPolicy.IGNORANCE);
		maxCost = edge.getCost(firstEto, secondEto, CostPolicy.MAXIMUM, RiskPolicy.IGNORANCE);
		assertEquals(continuum.getBaseCost() + 30d, minCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost() + 39d, avgCost, PrecisionDouble.UNIT_DECA_MICRO);
		assertEquals(continuum.getBaseCost() + 50d, maxCost, PrecisionDouble.UNIT_DECA_MICRO);
	
		// TODO: grid voxels
	}
	
	/**
	 * Tests risk policies for cost intervals.
	 */
	@Test
	public void testRiskPolicy() {
		// TODO: implement
	}
	
}
