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

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.geom.LineSegment;
import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;

import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Box;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.geom.Sphere;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;

/**
 * Performs obstacles tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class ObstaclesTest {
	
	/**
	 * Tests line segment intersections.
	 */
	@Test
	public void testLineSegment() {
		Vec4 a = new Vec4(1d, 0d, 0d);
		Vec4 b = new Vec4(-1d, 0d, 0d);
		Vec4 c = new Vec4(0d, 1d, 0d);
		
		LineSegment segment = new LineSegment(a, b);
		Sphere sphere1 = new Sphere(Vec4.ZERO, 1d);
		assertTrue(segment.intersects(sphere1));
		
		Sphere sphere2 = new Sphere(Vec4.ZERO, 0.5d);
		assertTrue(segment.intersects(sphere2));
		
		Sphere sphere3 = new Sphere(Vec4.ZERO, 1.5d);
		assertTrue(segment.intersects(sphere3));
		
		Sphere sphere4 = new Sphere(c, 0.5d);
		assertFalse(segment.intersects(sphere4));
		
		Sphere sphere5 = new Sphere(c, 1d);
		assertTrue(segment.intersects(sphere5));
		
		Sphere sphere6 = new Sphere(c, 1.5d);
		assertTrue(segment.intersects(sphere6));
	}
	
	/**
	 * Tests obstacle sphere intersections.
	 */
	@Test
	public void testObstacleSphere() {
		Earth earth = new Earth();
		Sector sector = new Sector(
				Angle.fromDegrees(-1d), Angle.fromDegrees(1d),
				Angle.fromDegrees(-1d), Angle.fromDegrees(1d));
		Box envBox = Sector.computeBoundingBox(earth, 1d, sector, 0d, 5000d);
		PlanningContinuum continuum = new PlanningContinuum(new com.cfar.swim.worldwind.geom.Box(envBox));
		continuum.setGlobe(earth);
		continuum.setNormalizer(envBox.getDiameter());
		ObstacleSphere sphere = new ObstacleSphere(Position.ZERO, 5d);
		sphere.setAltitudeDatum(AVKey.ABOVE_MEAN_SEA_LEVEL, AVKey.ABOVE_MEAN_SEA_LEVEL);
		// NOTE: a too large distance may result in a line below the obstacle elevation
		Position a = new Position(Angle.fromDegrees(0), Angle.fromDegrees(-0.01d), 0d);
		Position b = new Position(Angle.fromDegrees(0), Angle.fromDegrees(0.01d), 0d);
		Edge edge = new Edge(continuum, a, b);
		assertTrue(edge.intersects(sphere.getExtent(continuum.getGlobe())));
	}
	
}
