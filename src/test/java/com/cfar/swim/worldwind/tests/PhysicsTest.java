package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.awt.Dimension;

import javax.swing.JFrame;
import javax.vecmath.Matrix4f;
import javax.vecmath.Vector3f;

import org.junit.Test;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.narrowphase.GjkEpaSolver;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.cfar.swim.worldwind.geom.Box;
import com.jogamp.opengl.awt.GLJPanel;

import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Performs jBullet physics engine tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class PhysicsTest {

	/** the dynamics world of this physics test */
	private DynamicsWorld dynamicsWorld = null;
	
	/** the debug drawer of this phyiscs test */
	//private GLDebugDrawer debugDrawer = null;
	
	/** the frame containing the debug drawer of this physics test */
	private JFrame frame = new JFrame();
	
	/**
	 * Constructs a new jBullet physics engine test.
	 */
	public PhysicsTest() {
		GLJPanel canvas = new GLJPanel();
		canvas.setPreferredSize(new Dimension(600, 600));
		canvas.setBackground(Color.gray);
		canvas.setVisible(true);
		
		/*
		frame.setTitle("Physics Test");
		frame.getContentPane().add(canvas);
		frame.getContentPane().setPreferredSize(new Dimension(600, 600));
		frame.getContentPane().setVisible(true);
		frame.setPreferredSize(new Dimension(600, 600));
		frame.pack();
		frame.setVisible(true);
		*/
		
		CollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
		Dispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
		BroadphaseInterface broadphase = new DbvtBroadphase();
		ConstraintSolver solver = new SequentialImpulseConstraintSolver();
		dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		//this.debugDrawer = new GLDebugDrawer((GL2) canvas.getGL());
		//dynamicsWorld.setDebugDrawer(this.debugDrawer);
	}
	
	/**
	 * Tests collision shape (rigid body) collisions.
	 */
	@Test
	public void testCollisions() {
		GjkEpaSolver gjkSolver = new GjkEpaSolver();
		GjkEpaSolver.Results results = new GjkEpaSolver.Results();
		boolean intersects = false;
		
		// box
		// x, z, y
		Vector3f boxHalfExtents = new Vector3f(5f, 5f, 5f);
		BoxShape boxShape = new BoxShape(boxHalfExtents);
		Matrix4f btm = new Matrix4f();
		btm.setIdentity(); // no translation or rotation
		Transform boxTransform = new Transform(btm);
		//boxTransform.setIdentity();
		
		RigidBody boxBody = new RigidBody(0f, new DefaultMotionState(), boxShape);
		this.dynamicsWorld.addRigidBody(boxBody);
		
		// cylinder
		// radius, half-height, radius
		Vector3f cylinderHalfExtents = new Vector3f(5f, 5f, 5f);
		CylinderShape cylinderShape = new CylinderShape(cylinderHalfExtents);
		Matrix4f ctm = new Matrix4f();
		ctm.setIdentity(); // no translation or rotation
		Transform cylinderTransform = new Transform(ctm);
		
		RigidBody cylinderBody = new RigidBody(0f, new DefaultMotionState(), cylinderShape);
		this.dynamicsWorld.addRigidBody(cylinderBody);
		this.dynamicsWorld.debugDrawWorld();
		
		// box-cylinder intersection
		intersects = gjkSolver.collide(boxShape, boxTransform, cylinderShape, cylinderTransform, 0.04f, results);
		assertTrue(intersects);
		
		ctm.setTranslation(new Vector3f(5f, 0f, 0f));
		cylinderTransform.set(ctm);
		intersects = gjkSolver.collide(boxShape, boxTransform, cylinderShape, cylinderTransform, 0.04f, results);
		assertTrue(intersects);
		
		ctm.setTranslation(new Vector3f(10f, 10f, 0f));
		cylinderTransform.set(ctm);
		intersects = gjkSolver.collide(boxShape, boxTransform, cylinderShape, cylinderTransform, 0.04f, results);
		assertTrue(intersects);
		
		Box box = new Box(new Vec4(0d, 0d, 0d, 1d));
		Cylinder cylinder = new Cylinder(new Vec4(0d, 0d, -0.5d, 1d), new Vec4(0d, 0d, 0.5d, 1d), 0.5d);
		boxHalfExtents = new Vector3f(
				(float) box.getRLength() / 2,
				(float) box.getSLength() / 2,
				(float) box.getTLength() / 2);
		boxShape = new BoxShape(boxHalfExtents);
		boxTransform.setIdentity();
		
		cylinderHalfExtents = new Vector3f(
				(float) cylinder.getCylinderRadius(),
				(float) cylinder.getCylinderHeight() / 2,
				(float) cylinder.getCylinderRadius());
		cylinderShape = new CylinderShape(cylinderHalfExtents);
		cylinderTransform.setIdentity();
		
		intersects = gjkSolver.collide(boxShape, boxTransform, cylinderShape, cylinderTransform, 0.04f, results);
		assertTrue(intersects);
	}
	
}
