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

import static org.junit.Assert.assertTrue;

import java.time.ZonedDateTime;

import org.junit.Test;

import com.cfar.swim.worldwind.planners.cgs.adstar.ADStarWaypoint;
import com.cfar.swim.worldwind.planners.cgs.arastar.ARAStarWaypoint;
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;

/**
 * Realizes waypoint tests.
 * 
 * @author Stephan Heinemann
 */
public class WaypointTest {

	/**
	 * Tests the priority order of waypoints.
	 */
	@Test
	public void testPriorityOrder() {
		// A* waypoints
		AStarWaypoint aswp1 = new AStarWaypoint(new Position(Angle.ZERO, Angle.ZERO, 0));
		AStarWaypoint aswp2 = aswp1.clone();
		
		// lower f
		aswp1.setCost(0d);
		aswp1.setH(0d);
		aswp2.setCost(1d);
		aswp2.setH(0d);
		assertTrue(0 > aswp1.compareTo(aswp2));
		
		// higher f
		aswp1.setH(2d);
		aswp2.setH(0d);
		assertTrue(0 < aswp1.compareTo(aswp2));
		
		// equal f: break f-ties with higher g
		aswp1.setCost(0d);
		aswp2.setH(1d);
		assertTrue(0 < aswp1.compareTo(aswp2));
		
		// equal f and g
		aswp1.setCost(1d);
		aswp2.setH(2d);
		assertTrue(0 == aswp1.compareTo(aswp2));
		
		// equal f and g: break f-g-ties with earlier ETO
		aswp1.setEto(ZonedDateTime.now());
		aswp2.setEto(ZonedDateTime.now().plusNanos(1l));
		assertTrue(0 > aswp1.compareTo(aswp2));
		
		// ARA* waypoints
		ARAStarWaypoint araswp1 = new ARAStarWaypoint(aswp1);
		ARAStarWaypoint araswp2 = new ARAStarWaypoint(aswp2, 1d);
		
		// equal f, g and epsilon
		araswp1.setCost(0d);
		araswp1.setH(1d);
		araswp2.setCost(0d);
		araswp2.setH(1d);
		assertTrue(0 == araswp1.compareTo(araswp2));
		
		// lower epsilon
		araswp2.setEpsilon(2d);
		assertTrue(0 > araswp1.compareTo(araswp2));
		
		// higher epsilon
		araswp1.setEpsilon(3d);
		assertTrue(0 < araswp1.compareTo(araswp2));
		
		// AD* waypoints
		ADStarWaypoint adswp1 = new ADStarWaypoint(aswp1, 1d);
		adswp1.setCost(1d);
		adswp1.setH(1d);
		adswp1.setEto(ZonedDateTime.now());
		adswp1.makeConsistent();
		assertTrue(adswp1.isConsistent());
		
		// maintain all immutable members
		ADStarWaypoint adswp2 = (ADStarWaypoint) adswp1.clone();
		assertTrue(0 == adswp1.compareTo(adswp2));
		
		// under-consistency and priority order
		adswp1.setV(0.7d);
		assertTrue(adswp1.isUnderConsistent());
		adswp2.setV(0.5d);
		assertTrue(adswp2.isUnderConsistent());
		
		// equal key, under-consistent and higher v
		adswp1.setH(0.8d);
		assertTrue(0 > adswp1.compareTo(adswp2));
		
		// over-consistency and priority order
		adswp1.setV(1.7d);
		assertTrue(adswp1.isOverConsistent());
		adswp2.setV(1.5d);
		assertTrue(adswp2.isOverConsistent());
		
		// equal key, over-consistent and higher g
		adswp1.setG(1.2d);
		assertTrue(0 > adswp1.compareTo(adswp2));
		
		// equal key, over-consistent, equal g, equal ETO
		adswp2.setG(1.2d);
		adswp2.setH(0.8d);
		assertTrue(0 == adswp1.compareTo(adswp2));
		
		// equal key, over-consistent, equal g, earlier ETO
		adswp2.setEto(adswp2.getEto().plusNanos(1l));
		assertTrue(0 > adswp1.compareTo(adswp2));
	}
	
}
