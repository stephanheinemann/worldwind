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
package com.cfar.swim.worldwind.geom;

import javax.vecmath.AxisAngle4f;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.narrowphase.GjkEpaSolver;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.linearmath.Transform;
import com.cfar.swim.worldwind.geom.precision.Precision;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Box;
import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.Quaternion;
import gov.nasa.worldwind.geom.Sphere;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Provides methods to determine collisions between geometric extents based on
 * a GJK EPA solver.
 * 
 * @author Stephan Heinemann
 *
 */
public class Collisions {
	
	/** the solver to determine collisions between geometric extents */
	public static final GjkEpaSolver SOLVER = new GjkEpaSolver();
	
	/**
	 * Determines the closest angle body axis with respect to a reference axis.
	 * 
	 * @param axes the body axes
	 * @param axis the reference axis
	 * 
	 * @return the closest angle body axis
	 */
	private static Vec4 closestAngleBodyAxis(Vec4[] axes, Vec4 axis) {
		Vec4 closestAxis = axes[0];
		
		Angle rr = axis.angleBetween3(axes[0]);
		Angle rs = axis.angleBetween3(axes[1]);
		Angle rt = axis.angleBetween3(axes[2]);
		
		Angle closestAngle = rr;
		
		if (Math.abs(closestAngle.getDegrees()) > Math.abs(rs.getDegrees())) {
			closestAxis = axes[1];
			closestAngle = rs;
		}
		if (Math.abs(closestAngle.getDegrees()) > Math.abs(rt.getDegrees())) {
			closestAxis = axes[2];
		}
		
		return closestAxis;
	}
	
	/**
	 * Creates a box shape from a box.
	 * 
	 * @param box the box
	 * 
	 * @return the box shape of the box
	 */
	private static BoxShape createBoxShape(Box box) {
		Vec4[] boxAxes = {box.getRAxis(), box.getSAxis(), box.getTAxis()};
		Vec4[] boxRotationAxis = {Vec4.ZERO};
		Angle boxRotationAngle = Angle.ZERO;
		
		// determine x-axis length
		Vec4 boxXAxis = Collisions.closestAngleBodyAxis(boxAxes, Vec4.UNIT_X);
		double boxXLength = boxXAxis.getLength3();
		
		Quaternion boxXRotation = Quaternion.IDENTITY;
		if (!Angle.ZERO.equals(Vec4.UNIT_X.angleBetween3(boxXAxis))) {
			boxRotationAngle = Vec4.axisAngle(
					Vec4.UNIT_X, boxXAxis, boxRotationAxis);
			boxXRotation = Quaternion.fromAxisAngle(
					boxRotationAngle, boxRotationAxis[0]);
		}
		
		boxAxes[0] = boxAxes[0].transformBy3(boxXRotation);
		boxAxes[1] = boxAxes[1].transformBy3(boxXRotation);
		boxAxes[2] = boxAxes[2].transformBy3(boxXRotation);
		
		// determine y-axis length
		Vec4 boxYAxis = Collisions.closestAngleBodyAxis(boxAxes, Vec4.UNIT_Y);
		double boxYLength = boxYAxis.getLength3();
		
		// determine z-axis length
		double boxZLength = box.getTLength();
		if ((!boxXAxis.equals(box.getRAxis()))
				&& (!boxYAxis.equals(box.getRAxis().transformBy3(boxXRotation)))) {
			boxZLength = box.getRLength();
		} else if ((!boxXAxis.equals(box.getSAxis()))
				&& (!boxYAxis.equals(box.getSAxis().transformBy3(boxXRotation)))) {
			boxZLength = box.getSLength();
		}
		
		// jBullet uses a right-hand coordinate system with y as vertical axis
		Vector3f boxHalfExtents = new Vector3f(
				(float) (boxXLength / 2d),  // X
				(float) (boxZLength / 2d),  // Z
				(float) (boxYLength / 2d)); // Y
		return new BoxShape(boxHalfExtents);
	}
	
	/**
	 * Creates a transform from a box.
	 * 
	 * @param box the box
	 * 
	 * @return the transform of the box
	 */
	private static Transform createBoxTransform(Box box) {
		// box translation
		Vec4 boxTranslation = box.getCenter();
		// jBullet uses a right-hand coordinate system with y as vertical axis
		Vector3f translation = new Vector3f(
				(float) boxTranslation.getX(),
				(float) boxTranslation.getZ(),
				(float) boxTranslation.getY());
		
		// box rotation
		Vec4[] boxAxes = {box.getRAxis(), box.getSAxis(), box.getTAxis()};
		Vec4[] boxRotationAxis = {Vec4.ZERO};
		Angle boxRotationAngle = Angle.ZERO;
		
		// first rotation
		Vec4 boxXAxis = Collisions.closestAngleBodyAxis(boxAxes, Vec4.UNIT_X);
		Quaternion boxXRotation = Quaternion.IDENTITY;
		if (!Angle.ZERO.equals(Vec4.UNIT_X.angleBetween3(boxXAxis))) {
			boxRotationAngle = Vec4.axisAngle(
					Vec4.UNIT_X, boxXAxis, boxRotationAxis);
			boxXRotation = Quaternion.fromAxisAngle(
					boxRotationAngle, boxRotationAxis[0]);
		}
		
		boxAxes[0] = boxAxes[0].transformBy3(boxXRotation);
		boxAxes[1] = boxAxes[1].transformBy3(boxXRotation);
		boxAxes[2] = boxAxes[2].transformBy3(boxXRotation);
		
		// second rotation
		Vec4 boxYAxis = Collisions.closestAngleBodyAxis(boxAxes, Vec4.UNIT_Y);
		Quaternion boxYRotation = Quaternion.IDENTITY;
		if (!Angle.ZERO.equals(Vec4.UNIT_Y.angleBetween3(boxYAxis))) {
			boxRotationAngle = Vec4.axisAngle(
					Vec4.UNIT_Y, boxYAxis, boxRotationAxis);
			boxYRotation = Quaternion.fromAxisAngle(
					boxRotationAngle, boxRotationAxis[0]);
		}
		
		// sequence second rotation after first rotation
		Quaternion boxRotation = boxYRotation.multiply(boxXRotation);
		
		// jBullet uses a right-hand coordinate system with y as vertical axis
		Quat4f rotation = new Quat4f(
				(float) boxRotation.getX(),
				(float) boxRotation.getZ(),
				(float) boxRotation.getY(),
				(float) boxRotation.getW());
		
		// obtain box transformation (translation and rotation)
		Matrix4f btm = new Matrix4f();
		btm.setIdentity(); // no translation or rotation
		btm.setTranslation(translation); // center translation
		btm.setRotation(rotation); // sequenced rotation
		
		return new Transform(btm);
	}
	
	/**
	 * Creates a cylinder shape from a cylinder.
	 * 
	 * @param cylinder the cylinder
	 * 
	 * @return the cylinder shape of the cylinder
	 */
	private static CylinderShape createCylinderShape(Cylinder cylinder) {
		// jBullet uses a right-hand coordinate system with y as vertical axis
		Vector3f cylinderHalfExtents = new Vector3f(
				(float) cylinder.getCylinderRadius(),
				(float) (cylinder.getCylinderHeight() / 2d),
				(float) cylinder.getCylinderRadius());
		return new CylinderShape(cylinderHalfExtents);
	}
	
	/**
	 * Creates a transform from a cylinder.
	 * 
	 * @param cylinder the cylinder
	 * 
	 * @return the transform of the cylinder
	 */
	private static Transform createCylinderTransform(Cylinder cylinder) {
		// cylinder translation
		Vec4 cylinderTranslation = cylinder.getCenter();
		// jBullet uses a right-hand coordinate system with y as vertical axis
		Vector3f translation = new Vector3f(
				(float) cylinderTranslation.getX(),
				(float) cylinderTranslation.getZ(),
				(float) cylinderTranslation.getY());
		
		// cylinder rotation
		Vec4[] cylinderRotationAxis = new Vec4[] {Vec4.ZERO};
		Angle cylinderRotationAngle = Angle.ZERO;
		
		if (!Angle.ZERO.equals(Vec4.UNIT_Z.angleBetween3(
				cylinder.getAxisUnitDirection()))) {
			cylinderRotationAngle = Vec4.axisAngle(
					Vec4.UNIT_Z,
					cylinder.getAxisUnitDirection(),
					cylinderRotationAxis);
		}
		
		// jBullet uses a right-hand coordinate system with y as vertical axis
		AxisAngle4f rotationAxisAngle = new AxisAngle4f(
				(float) cylinderRotationAxis[0].getX(),
				(float) cylinderRotationAxis[0].getZ(),
				(float) cylinderRotationAxis[0].getY(),
				(float) cylinderRotationAngle.getRadians());
		// obtain cylinder rotation quaternion
		Quat4f rotation = new Quat4f();
		rotation.set(rotationAxisAngle);
		
		// obtain cylinder transformation (translation and rotation)
		Matrix4f ctm = new Matrix4f();
		ctm.setIdentity(); // no translation or rotation
		ctm.setTranslation(translation); // center translation
		ctm.setRotation(rotation); // vertical rotation
		
		return new Transform(ctm);
	}
	
	/**
	 * Creates a sphere shape from a sphere.
	 * 
	 * @param sphere the sphere
	 * 
	 * @return the sphere shape of the sphere
	 */
	private static SphereShape createSphereShape(Sphere sphere) {
		// jBullet uses a right-hand coordinate system with y as vertical axis
		return new SphereShape((float) sphere.getRadius());
	}
	
	/**
	 * Creates a transform from a sphere.
	 * 
	 * @param sphere the sphere
	 * 
	 * @return the transform of the sphere
	 */
	private static Transform createSphereTransform(Sphere sphere) {
		// sphere translation
		Vec4 sphereTranslation = sphere.getCenter();
		// jBullet uses a right-hand coordinate system with y as vertical axis
		Vector3f translation = new Vector3f(
				(float) sphereTranslation.getX(),
				(float) sphereTranslation.getZ(),
				(float) sphereTranslation.getY());
		
		// obtain sphere transformation (translation without rotation)
		Matrix4f stm = new Matrix4f();
		stm.setIdentity(); // no translation or rotation
		stm.setTranslation(translation); // center translation
		
		return new Transform(stm);
	}
	
	/**
	 * Determines whether two geometric extents collide.
	 * 
	 * @param extent1 the first geometric extent
	 * @param extent2 the second geometric extent
	 * 
	 * @return true if the two geometric extents collide, false otherwise
	 * 
	 * @throws IllegalArgumentException if the either extent is invalid
	 */
	public static boolean collide(Extent extent1, Extent extent2) {
		boolean collide = false;
		
		if (extent1 instanceof Box) {
			collide = Collisions.collide((Box) extent1, extent2);
		} else if (extent1 instanceof Cylinder) {
			collide = Collisions.collide((Cylinder) extent1, extent2);
		} else if (extent1 instanceof Sphere) {
			collide = Collisions.collide((Sphere) extent1, extent2);
		} else {
			throw new IllegalArgumentException("extent is invalid");
		}
		
		return collide;
	}
	
	/**
	 * Determines whether a geometric box and extent collide.
	 * 
	 * @param box the geometric box
	 * @param extent the geometric extent
	 * 
	 * @return true if the geometric box and extent collide, false otherwise
	 * 
	 * @throws IllegalArgumentException if the extent is invalid
	 */
	public static boolean collide(Box box, Extent extent) {
		boolean collide = false;
		
		if (extent instanceof Box) {
			collide = Collisions.collide(box, (Box) extent);
		} else if (extent instanceof Cylinder) {
			collide = Collisions.collide(box, (Cylinder) extent);
		} else if (extent instanceof Sphere) {
			collide = Collisions.collide(box, (Sphere) extent);
		} else {
			throw new IllegalArgumentException("extent is invalid");
		}
		
		return collide;
	}
	
	/**
	 * Determines whether a geometric cylinder and extent collide.
	 * 
	 * @param cylinder the geometric cylinder
	 * @param extent the geometric extent
	 * 
	 * @return true if the geometric cylinder and extent collide,
	 *         false otherwise
	 * 
	 * @throws IllegalArgumentException if the extent is invalid
	 */
	public static boolean collide(Cylinder cylinder, Extent extent) {
		boolean collide = false;
		
		if (extent instanceof Box) {
			collide = Collisions.collide((Box) extent, cylinder);
		} else if (extent instanceof Cylinder) {
			collide = Collisions.collide(cylinder, (Cylinder) extent);
		} else if (extent instanceof Sphere) {
			collide = Collisions.collide(cylinder, (Sphere) extent);
		} else {
			throw new IllegalArgumentException("extent is invalid");
		}
		
		return collide;
	}
	
	/**
	 * Determines whether a geometric sphere and extent collide.
	 * 
	 * @param sphere the geometric sphere
	 * @param extent the geometric extent
	 * 
	 * @return true if the geometric sphere and extent collide, false otherwise
	 * 
	 * @throws IllegalArgumentException if the extent is invalid
	 */
	public static boolean collide(Sphere sphere, Extent extent) {
		boolean collide = false;
		
		if (extent instanceof Box) {
			collide = Collisions.collide((Box) extent, sphere);
		} else if (extent instanceof Cylinder) {
			collide = Collisions.collide((Cylinder) extent, sphere);
		} else if (extent instanceof Sphere) {
			collide = Collisions.collide(sphere, (Sphere) extent);
		} else {
			throw new IllegalArgumentException("extent is invalid");
		}
		
		return collide;
	}
	
	/**
	 * Determines whether two geometric boxes collide.
	 * 
	 * @param box1 the first geometric box
	 * @param box2 the second geometric box
	 * 
	 * @return true if the two geometric boxes collide, false otherwise
	 */
	public static boolean collide(Box box1, Box box2) {
		GjkEpaSolver.Results results = new GjkEpaSolver.Results();
		
		BoxShape boxShape1 = Collisions.createBoxShape(box1);
		Transform boxTransform1 = Collisions.createBoxTransform(box1);
		
		BoxShape boxShape2 = Collisions.createBoxShape(box2);
		Transform boxTransform2 = Collisions.createBoxTransform(box2);
		
		// perform intersection GJK test
		return Collisions.SOLVER.collide(
				boxShape1, boxTransform1, boxShape2, boxTransform2,
				(float) Precision.EPSILON, results);
	}
	
	/**
	 * Determines whether a geometric box and cylinder collide.
	 * 
	 * @param box the geometric box
	 * @param cylinder the geometric cylinder
	 * 
	 * @return true if the geometric box and cylinder collide, false otherwise
	 */
	public static boolean collide(Box box, Cylinder cylinder) {
		GjkEpaSolver.Results results = new GjkEpaSolver.Results();
		
		BoxShape boxShape = Collisions.createBoxShape(box);
		Transform boxTransform = Collisions.createBoxTransform(box);
		
		CylinderShape cylinderShape = Collisions.createCylinderShape(cylinder);
		Transform cylinderTransform = Collisions.createCylinderTransform(cylinder);
		
		// perform intersection GJK test
		return Collisions.SOLVER.collide(
				boxShape, boxTransform, cylinderShape, cylinderTransform,
				(float) Precision.EPSILON, results);
	}
	
	/**
	 * Determines whether a geometric box and sphere collide.
	 * 
	 * @param box the geometric box
	 * @param sphere the geometric sphere
	 * 
	 * @return true if the geometric box and sphere collide, false otherwise
	 */
	public static boolean collide(Box box, Sphere sphere) {
		GjkEpaSolver.Results results = new GjkEpaSolver.Results();
		
		BoxShape boxShape = Collisions.createBoxShape(box);
		Transform boxTransform = Collisions.createBoxTransform(box);
		
		SphereShape sphereShape = Collisions.createSphereShape(sphere);
		Transform sphereTransform = Collisions.createSphereTransform(sphere);
		
		// perform intersection GJK test
		return Collisions.SOLVER.collide(
				boxShape, boxTransform, sphereShape, sphereTransform,
				(float) Precision.EPSILON, results);
	}
	
	/**
	 * Determines whether two geometric cylinders collide.
	 * 
	 * @param cylinder1 the first geometric cylinder
	 * @param cylinder2 the second geometric cylinder
	 * 
	 * @return true if the two geometric cylinders collide, false otherwise
	 */
	public static boolean collide(Cylinder cylinder1, Cylinder cylinder2) {
		GjkEpaSolver.Results results = new GjkEpaSolver.Results();
		
		CylinderShape cylinderShape1 = Collisions.createCylinderShape(cylinder1);
		Transform cylinderTransform1 = Collisions.createCylinderTransform(cylinder1);
		
		CylinderShape cylinderShape2 = Collisions.createCylinderShape(cylinder2);
		Transform cylinderTransform2 = Collisions.createCylinderTransform(cylinder2);
		
		// perform intersection GJK test
		return Collisions.SOLVER.collide(
				cylinderShape1, cylinderTransform1, cylinderShape2, cylinderTransform2,
				(float) Precision.EPSILON, results);
	}
	
	/**
	 * Determines whether a geometric cylinder and sphere collide.
	 * 
	 * @param cylinder the geometric cylinder
	 * @param sphere the geometric sphere
	 * 
	 * @return true if the geometric cylinder and sphere collide,
	 *         false otherwise
	 */
	public static boolean collide(Cylinder cylinder, Sphere sphere) {
		GjkEpaSolver.Results results = new GjkEpaSolver.Results();
		
		CylinderShape cylinderShape = Collisions.createCylinderShape(cylinder);
		Transform cylinderTransform = Collisions.createCylinderTransform(cylinder);
		
		SphereShape sphereShape = Collisions.createSphereShape(sphere);
		Transform sphereTransform = Collisions.createSphereTransform(sphere);
		
		// perform intersection GJK test
		return Collisions.SOLVER.collide(
				cylinderShape, cylinderTransform, sphereShape, sphereTransform,
				(float) Precision.EPSILON, results);
	}
	
	/**
	 * Determines whether two geometric spheres collide.
	 * 
	 * @param sphere1 the first geometric sphere
	 * @param sphere2 the second geometric sphere
	 * 
	 * @return true if the two geometric spheres collide, false otherwise
	 */
	public static boolean collide(Sphere sphere1, Sphere sphere2) {
		GjkEpaSolver.Results results = new GjkEpaSolver.Results();
		
		SphereShape sphereShape1 = Collisions.createSphereShape(sphere1);
		Transform sphereTransform1 = Collisions.createSphereTransform(sphere1);
		
		SphereShape sphereShape2 = Collisions.createSphereShape(sphere2);
		Transform sphereTransform2 = Collisions.createSphereTransform(sphere2);
		
		// perform intersection GJK test
		return Collisions.SOLVER.collide(
				sphereShape1, sphereTransform1, sphereShape2, sphereTransform2,
				(float) Precision.EPSILON, results);
	}
	
}
