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

import java.awt.BorderLayout;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.List;

import javax.xml.bind.JAXBException;

import org.junit.Test;
import org.xml.sax.InputSource;

import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.data.iwxxm.IwxxmUpdater;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.javafx.PlanningTimePicker;
import com.cfar.swim.worldwind.javafx.SwimDataListView;
import com.cfar.swim.worldwind.javafx.ThresholdCostSlider;
import com.cfar.swim.worldwind.planners.cgs.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.thetastar.ThetaStarPlanner;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.BasicModel;
import gov.nasa.worldwind.Configuration;
import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.awt.WorldWindowGLCanvas;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.LayerList;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.GlobeAnnotation;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Path;
import javafx.embed.swing.JFXPanel;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.paint.Color;

/**
 * Performs regular grid tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class RegularGridTest {
	
	static Model model;
	static PlanningGrid uvicGrid;
	static PlanningGrid largeGrid;
	static PlanningGrid tsGrid;
	static PlanningGrid tcGrid;
	static Iris iris;
	
	// dirty, dirty, dirty...
	static SwimDataListView list;
	
	// TODO: proper initialization with a sequence of tests
	private static class AppFrame extends javax.swing.JFrame {
        
		private static final long serialVersionUID = 1L;
		
		public JFXPanel timePanel;

		public AppFrame() {
			try {
        	WorldWindowGLCanvas wwd = new WorldWindowGLCanvas();
            wwd.setPreferredSize(new java.awt.Dimension(1000, 800));
            this.getContentPane().add(wwd, java.awt.BorderLayout.CENTER);
            this.pack();
            wwd.setModel(new BasicModel());
            model = wwd.getModel();
            
            Configuration.setValue(
            		AVKey.MIL_STD_2525_ICON_RETRIEVER_PATH,
            		ClassLoader.getSystemResource("milstd2525"));
        
            RenderableLayer renderableLayer = new RenderableLayer();
            LayerList layers = wwd.getModel().getLayers();
            layers.add(renderableLayer);
            
            
            // TODO: separate test procedures
            Sector uvic = new Sector(
            	Angle.fromDegrees(48.462836),
            	Angle.fromDegrees(48.463418),
            	Angle.fromDegrees(-123.312186),
        		Angle.fromDegrees(-123.310799)
            	);
            gov.nasa.worldwind.geom.Box uvicBox = Sector.computeBoundingBox(wwd.getModel().getGlobe(), 1.0, uvic, 0.0, 150.0);
            //uvicGrid = new NonUniformCostIntervalGrid(new com.cfar.swim.worldwind.geom.Box(uvicBox));
            com.cfar.swim.worldwind.geom.Box uvicB = new com.cfar.swim.worldwind.geom.Box(uvicBox);
            Cube uvicCube = new Cube(uvicB.getOrigin(), uvicB.getUnitAxes(), uvicB.getRLength() / 10);
            uvicGrid = new PlanningGrid(uvicCube, 10, 10, 5);
            uvicGrid.setThreshold(0);
            //uvicGrid.addChildren(uvicBox.getTLength() / 5.0);
            //uvicGrid.setThresholdCost(50);
            //uvicGrid.setGlobe(model.getGlobe());
            renderableLayer.addRenderable(uvicGrid);
            
            Sector large = new Sector(
            	Angle.fromDegrees(0.0),
            	Angle.fromDegrees(90.0),
            	Angle.fromDegrees(0.0),
            	Angle.fromDegrees(90.0));
            gov.nasa.worldwind.geom.Box largeBox = Sector.computeBoundingBox(wwd.getModel().getGlobe(), 1.0, large, 0.0, 500000.0);
            //largeGrid = new NonUniformCostIntervalGrid(new com.cfar.swim.worldwind.geom.Box(largeBox));
            com.cfar.swim.worldwind.geom.Box largeB = new com.cfar.swim.worldwind.geom.Box(largeBox);
            Cube largeCube = new Cube(largeB.getOrigin(), largeB.getUnitAxes(), largeB.getRLength() / 10);
            largeGrid = new PlanningGrid(largeCube, 10, 10, 5);
            largeGrid.setThreshold(0);
            //largeGrid.addChildren(largeBox.getTLength() / 4.0);
            largeGrid.addChildren(0, 8, 3, 2, 2, 2);
            largeGrid.addChildren(1, 9, 3, 2, 2, 2);
            largeGrid.addChildren(0, 9, 1, 2, 2, 2);
            largeGrid.addChildren(2, 9, 1, 2, 2, 2);
            //largeGrid.setGlobe(model.getGlobe());
            renderableLayer.addRenderable(largeGrid);
            
            Sector ts = new Sector(
                	Angle.fromDegrees(50.0),
                	Angle.fromDegrees(60.0),
                	Angle.fromDegrees(-15.0),
                	Angle.fromDegrees(5.0));
            gov.nasa.worldwind.geom.Box tsBox = Sector.computeBoundingBox(wwd.getModel().getGlobe(), 1.0, ts, 0.0, 500000.0);
            //renderableLayer.addRenderable(tsBox);
            //renderableLayer.addRenderable(new com.cfar.swim.worldwind.geom.Box(tsBox));
            //tsGrid = new NonUniformCostIntervalGrid(new com.cfar.swim.worldwind.geom.Box(tsBox));
            com.cfar.swim.worldwind.geom.Box tsB = new com.cfar.swim.worldwind.geom.Box(tsBox);
            Cube tsCube = new Cube(tsB.getOrigin(), tsB.getUnitAxes(), tsB.getRLength() / 10);
            tsGrid = new PlanningGrid(tsCube, 10, 10, 5);
            tsGrid.setThreshold(0);
            //tsGrid.addChildren(tsGrid.getTLength() / 4.0);
            //tsGrid.addChildren(3, 3, 0, 2, 2, 2);
            renderableLayer.addRenderable(tsGrid);
            
            /*
            Sector tc = new Sector(
                	Angle.fromDegrees(25.0),
                	Angle.fromDegrees(30.0),
                	Angle.fromDegrees(-75.0),
                	Angle.fromDegrees(-70.0));
            gov.nasa.worldwind.geom.Box tcBox = Sector.computeBoundingBox(wwd.getModel().getGlobe(), 1.0, tc, 0.0, 50000.0);
            //tcGrid = new NonUniformCostIntervalGrid(new com.cfar.swim.worldwind.geom.Box(tcBox));
            com.cfar.swim.worldwind.geom.Box tcB = new com.cfar.swim.worldwind.geom.Box(tcBox);
            Cube tcCube = new Cube(tcB.getOrigin(), tcB.getUnitAxes(), tcB.getRLength() / 10);
            tcGrid = new PlanningGrid(tcCube, 10, 10, 5);
            tcGrid.setThreshold(0);
            //tcGrid.addChildren(tcGrid.getTLength() / 4.0);
            renderableLayer.addRenderable(tcGrid);
            */
            
            iris = new Iris(new Position(ts.getCentroid(), 50000), 5000, CombatIdentification.FRIEND);
            iris.setCostInterval(new CostInterval(
            				"Iris",
            				ZonedDateTime.now(ZoneId.of("UTC")).minusYears(10),
            				ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
            				70));
            iris.getDepiction().setDesignation("Iris");
            //iris.getDepiction().setAnnotation(new DepictionAnnotation("Iris", iris.getCenter()));
            renderableLayer.addRenderable(iris);
            
            A320 a320 = new A320(new Position(ts.getCorners()[0], 50000), 5000, CombatIdentification.FRIEND);
            a320.setCostInterval(new CostInterval(
    				"A320",
    				ZonedDateTime.now(ZoneId.of("UTC")).minusYears(10),
    				ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
    				70));
            a320.getDepiction().setDesignation("A320");
            //a320.getDepiction().setAnnotation(new DepictionAnnotation("A320", iris.getCenter()));
            renderableLayer.addRenderable(a320);
            
            // TODO: add time slider with steps between min and max time (set using calender-like input)
            /*
            this.timePanel = new JFXPanel();
        	this.timePanel.setSize(300, 400);
        	this.timePanel.setScene(createScene(wwd));
        	this.getContentPane().add(this.timePanel, BorderLayout.WEST);
			*/
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		
		protected Scene createScene(WorldWindow worldWindow) {
        	Group group = new Group();
        	Scene scene = new Scene(group, Color.GREY);
        	PlanningTimePicker ptp = new PlanningTimePicker(worldWindow);
        	group.getChildren().add(ptp);
        	
        	ThresholdCostSlider slider = new ThresholdCostSlider(worldWindow);
        	slider.setShowTickMarks(true);
        	slider.setBlockIncrement(1.0);
        	slider.setLayoutY(300);
        	slider.setLayoutX(20);
        	slider.setPrefWidth(260);
        	group.getChildren().add(slider);
        	
        	Label label = new Label("Threshold Cost"/*, slider*/);
        	label.setLayoutY(280);
        	label.setLayoutX(20);
        	group.getChildren().add(label);
        	
        	list = new SwimDataListView(worldWindow);
        	list.setLayoutY(350);
        	list.setLayoutX(20);
        	list.setPrefHeight(300);
        	group.getChildren().add(list);
        	
        	return scene;
        }
		
    }
	
	//@Test
	public void test() throws InvocationTargetException, InterruptedException {
		AppFrame frame = new AppFrame();
		frame.setVisible(true);
		while (frame.isVisible()) {
			Thread.sleep(1000);
		}
		
		assertTrue(true);
	}
	
	@Test
	public void iwxxmUpdate() throws JAXBException, FileNotFoundException, InvocationTargetException, InterruptedException {
		AppFrame frame = new AppFrame();
		frame.setVisible(true);
		
		try {
		Layer layer = model.getLayers().getLayersByClass(RenderableLayer.class).get(0);
		
		uvicGrid.setGlobe(model.getGlobe());
		largeGrid.setGlobe(model.getGlobe());
		//IwxxmUpdater largeUpdater = new IwxxmUpdater(model, largeGrid);
		//largeUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1a-TS2.xml")));
		
		
		tsGrid.setGlobe(model.getGlobe());
		/*
		IwxxmUpdater tsUpdater = new IwxxmUpdater(model, tsGrid);
		tsUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1a-TS.xml")));
		tsUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1b-TS.xml")));
		tsUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1b-CNL.xml")));
		*/
		
		//tcGrid.setGlobe(model.getGlobe());
		
		/*
		IwxxmUpdater tcUpdater = new IwxxmUpdater(model, tcGrid);
		tcUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-2-TC.xml")));
		*/
		
		//list.registerDataActivationListerner(largeUpdater);
		//list.registerDataActivationListerner(tsUpdater);
		//list.registerDataActivationListerner(tcUpdater);
		
		// TODO: this seems to be the expected rst versus xyz bug
		/*
		System.out.println("iris location " + iris.getLocation());
		System.out.println("checking position " + iris.getCenter());
		Vec4 point = tsGrid.getGlobe().computePointFromPosition(iris.getCenter());
		System.out.println("cartesian point " + point);
		System.out.println("grid contains point " + tsGrid.contains(point));
		System.out.println("grid contains center " + tsGrid.contains(tsGrid.getCenter()));
		System.out.println("grid contains corner0 " + tsGrid.contains(tsGrid.getCorners()[0]));
		System.out.println("grid contains corner1 " + tsGrid.contains(tsGrid.getCorners()[1]));
		System.out.println("grid contains corner2 " + tsGrid.contains(tsGrid.getCorners()[2]));
		System.out.println("grid contains corner3 " + tsGrid.contains(tsGrid.getCorners()[3]));
		System.out.println("grid contains corner4 " + tsGrid.contains(tsGrid.getCorners()[4]));
		System.out.println("grid contains corner5 " + tsGrid.contains(tsGrid.getCorners()[5]));
		System.out.println("grid contains corner6 " + tsGrid.contains(tsGrid.getCorners()[6]));
		System.out.println("grid contains corner7 " + tsGrid.contains(tsGrid.getCorners()[7]));
		Vec4 zero = tsGrid.getGlobe().computePointFromPosition(new Position(LatLon.fromDegrees(0, 0), 0));
		System.out.println("grid contains zero " + tsGrid.contains(zero));
		
		System.out.println("grid contains center " + tcGrid.contains(tcGrid.getCenter()));
		System.out.println("grid contains center " + largeGrid.contains(largeGrid.getCenter()));
		
		Sphere sphere = new Sphere(point, 5000);
    	((RenderableLayer) layer).addRenderable(sphere);
		
		Set<? extends NonUniformCostIntervalGrid> cells = tsGrid.lookupCells(iris.getCenter());
		System.out.println("found iris cells " + cells.size());
		*/
		//Set<Position> neighbors = tsGrid.getNeighbors(iris.getCenter());
		//Set<Position> neighbors = tsGrid.getNeighbors(model.getGlobe().computePositionFromPoint(tsGrid.getChild(2, 2, 2).getBottomCenter()));
		/*
		Position position = model.getGlobe().computePositionFromPoint(tsGrid.getChild(2, 5, 1).getCorners()[0]);
		ObstacleSphere posIris = new ObstacleSphere(position, 2500);
    	posIris.setCostInterval(new CostInterval(
				"Position",
				ZonedDateTime.now(ZoneId.of("UTC")).minusYears(10),
				ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
				70));
    	((RenderableLayer) layer).addRenderable(posIris);
		
		Set<Position> neighbors = tsGrid.getNeighbors(position);
		
		System.out.println("neighbors size = " + neighbors.size());
		System.out.println("neighbors = " + neighbors);
        for (Position neighbor : neighbors) {
        	ObstacleSphere neighborIris = new ObstacleSphere(neighbor, 2500);
        	neighborIris.setCostInterval(new CostInterval(
    				"Neighbor",
    				ZonedDateTime.now(ZoneId.of("UTC")).minusYears(10),
    				ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
    				70));
        	//Layer layer = model.getLayers().getLayersByClass(RenderableLayer.class).get(0);
        	((RenderableLayer) layer).addRenderable(neighborIris);
        	
        	double firstCost = tsGrid.getStepCost(
        			position,
        			neighbor,
        			ZonedDateTime.now(ZoneId.of("UTC")).minusYears(10),
        			ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
        			CostPolicy.MAXIMUM);
        	double secondCost = tsGrid.getStepCost(
        			position,
        			neighbor,
        			ZonedDateTime.now(ZoneId.of("UTC")),
        			ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
        			CostPolicy.MAXIMUM);
        	System.out.println("first cost = " + firstCost + ", second cost = " + secondCost);
        }
        
        NonUniformCostIntervalGrid irisCell = tsGrid.lookupCells(iris.getCenter()).iterator().next();
        Set<? extends RegularGrid> irisNeighbors = irisCell.getNeighbors();
        for (RegularGrid irisNeighbor : irisNeighbors) {
        	((NonUniformCostIntervalGrid) irisNeighbor).addCostInterval(new CostInterval(
    				"neighbor cell",
    				ZonedDateTime.now(ZoneId.of("UTC")).minusYears(10),
    				ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
    				50));
        }
        */
        
        /*
        Vec4 x = new Vec4(1d, 0d, 0d);
        Vec4 y = new Vec4(0d, 1d, 0d);
        Vec4 z = new Vec4(0d, 0d, 1d);
        Vec4[] axes = {x, y, z};
        */
        
		/*
        Cube cube = new Cube(model.getGlobe().computePointFromPosition(iris.getCenter()), axes, 500000);
        ((RenderableLayer) layer).addRenderable(cube);
        */
        /*
		Sphere sphere = new Sphere(tsGrid.getOrigin(), 50000);
		((RenderableLayer) layer).addRenderable(sphere);
		*/
		
		/*
		Vec4[] axes = new Vec4[] {tsGrid.getUnitRAxis(), tsGrid.getUnitSAxis(), tsGrid.getUnitTAxis()};
        Cube cube = new Cube(tsGrid.getOrigin(), axes, 50000);
        CubicGrid cubicGrid = new CubicGrid(cube, 10, 5, 5);
        ((RenderableLayer) layer).addRenderable(cubicGrid);
        */
        
        /*
        Sphere tcsphere = new Sphere(tcGrid.getOrigin(), 50000);
		((RenderableLayer) layer).addRenderable(tcsphere);
        
        Vec4[] tcaxes = new Vec4[] {tcGrid.getUnitRAxis(), tcGrid.getUnitSAxis(), tcGrid.getUnitTAxis()};
        Cube tccube = new Cube(tcGrid.getOrigin(), tcaxes, 50000);
        CubicGrid tccubicGrid = new CubicGrid(tccube, 10, 5, 5);
        ((RenderableLayer) layer).addRenderable(tccubicGrid);
        
        Sphere lsphere = new Sphere(largeGrid.getOrigin(), 50000);
		((RenderableLayer) layer).addRenderable(lsphere);
        
        Vec4[] laxes = new Vec4[] {largeGrid.getUnitRAxis(), largeGrid.getUnitSAxis(), largeGrid.getUnitTAxis()};
        Cube lcube = new Cube(largeGrid.getOrigin(), laxes, 50000);
        CubicGrid lcubicGrid = new CubicGrid(lcube, 10, 5, 5);
        ((RenderableLayer) layer).addRenderable(lcubicGrid);
        */
        
		//Position origin = tsGrid.getCornerPositions()[0];
		//Position destination = tsGrid.getCornerPositions()[6];
		
		/*
		Sphere originSphere = new Sphere(tsGrid.getCorners()[0], 50000);
		Sphere destinationSphere = new Sphere(tsGrid.getCorners()[6], 50000);
		((RenderableLayer) layer).addRenderable(originSphere);
		((RenderableLayer) layer).addRenderable(destinationSphere);
		Vec4 pv = new Vec4(-889843.9756064053, 5581603.786233872, 3111118.000796212, 1.0);
		Sphere ps = new Sphere(pv, 5000);
		((RenderableLayer) layer).addRenderable(ps);
		*/
		
		/*
		ForwardAStarPlanner planner = new ForwardAStarPlanner(iris, tsGrid);
		Trajectory trajectory = planner.plan(origin, destination, ZonedDateTime.now());
		trajectory.setVisible(true);
		trajectory.setShowPositions(true);
		trajectory.setDrawVerticals(true);
		trajectory.setAttributes(new BasicShapeAttributes());
		trajectory.getAttributes().setOutlineMaterial(Material.MAGENTA);
		trajectory.getAttributes().setOutlineWidth(5d);
		trajectory.getAttributes().setOutlineOpacity(0.5d);
		
		for (Waypoint positionEstimate : (Iterable<Waypoint>) trajectory.getWaypoints()) {
			System.out.println(positionEstimate + " at " + positionEstimate.getEto());
		}
		((RenderableLayer) layer).addRenderable(trajectory);
		
		List<Position> waypoints = Arrays.asList(tsGrid.getCornerPositions());
		//ArrayList<Position> wp = new ArrayList<Position>(waypoints);
		//wp.add(tsGrid.getCenterPosition());
		trajectory = planner.plan(origin, destination, waypoints, ZonedDateTime.now());
		trajectory.setVisible(true);
		trajectory.setShowPositions(true);
		trajectory.setDrawVerticals(true);
		trajectory.setAttributes(new BasicShapeAttributes());
		trajectory.getAttributes().setOutlineMaterial(Material.GREEN);
		trajectory.getAttributes().setOutlineWidth(5d);
		trajectory.getAttributes().setOutlineOpacity(0.5d);
		((RenderableLayer) layer).addRenderable(trajectory);
		
		// A*
		Position paris = new Position(Angle.fromDegrees(48.864716d), Angle.fromDegrees(2.349014d), 10000);
		Position goal = tsGrid.getChild(1, 3, 1).getCenterPosition();
		A320 a320 = new A320(paris, 5000, CombatIdentification.FRIEND);
		ZonedDateTime start = ZonedDateTime.of(2012, 8, 10, 14, 0, 0, 0, ZoneId.of("UTC"));
		ForwardAStarPlanner planner2 = new ForwardAStarPlanner(a320, tsGrid);
		Trajectory trajectory2 = planner2.plan(paris, goal, start);
		trajectory2.setVisible(true);
		trajectory2.setShowPositions(true);
		trajectory2.setDrawVerticals(true);
		trajectory2.setAttributes(new BasicShapeAttributes());
		trajectory2.getAttributes().setOutlineMaterial(Material.ORANGE);
		trajectory2.getAttributes().setOutlineWidth(5d);
		trajectory2.getAttributes().setOutlineOpacity(0.5d);
		((RenderableLayer) layer).addRenderable(trajectory2);
		
		// Theta*
		for (Waypoint p : (Iterable<Waypoint>) trajectory2.getWaypoints()) {
			GlobeAnnotation ga = new GlobeAnnotation(p.getEto().toString() , p);
			((RenderableLayer) layer).addRenderable(ga);
		}
		
		ThetaStarPlanner planner3 = new ThetaStarPlanner(a320, tsGrid);
		Path path3 = planner3.plan(paris, goal, start);
		path3.setVisible(true);
		path3.setShowPositions(true);
		path3.setDrawVerticals(true);
		path3.setAttributes(new BasicShapeAttributes());
		path3.getAttributes().setOutlineMaterial(Material.RED);
		path3.getAttributes().setOutlineWidth(5d);
		path3.getAttributes().setOutlineOpacity(0.5d);
		((RenderableLayer) layer).addRenderable(path3);
		*/
		
		/*
		Set<? extends PlanningGrid> ics = largeGrid.getIntersectedCells(
				largeGrid.getBottomCenter(),
				largeGrid.getTopCenter(),
				2);
		System.out.println("intersected cells " + ics.size());
		for (PlanningGrid ic : ics) {
			ic.addCostInterval(new CostInterval(
					"ic",
					ZonedDateTime.now().minusYears(10),
					ZonedDateTime.now().plusYears(10),
					65));
		}
		*/
		
		//Set<PlanningGrid> intersectedCells = new HashSet<PlanningGrid>();
		//PlanningGrid child1 = largeGrid.getChild(0, 9, 0);
		//PlanningGrid child2 = largeGrid.getChild(0, 9, 4);
		//Position o1 = largeGrid.getGlobe().computePositionFromPoint(/*largeGrid.getCorners()[0]*//*child1.getCorners()[5]*/largeGrid.getBottomCenter());
		//Position o2 = largeGrid.getGlobe().computePositionFromPoint(/*largeGrid.getCorners()[6]*/child2.getCorners()[5]/*largeGrid.getBottomCenter()*/);
		/*
		Path ip = new Path(largeGrid.getIntersectedPositions(o1, o2));
		ip.setVisible(true);
		ip.setShowPositions(true);
		ip.setAttributes(new BasicShapeAttributes());
		ip.getAttributes().setOutlineMaterial(Material.MAGENTA);
		ip.getAttributes().setOutlineWidth(5d);
		ip.getAttributes().setOutlineOpacity(0.5d);
		((RenderableLayer) layer).addRenderable(ip);
		
		for (Position p : ip.getPositions()) {
			System.out.println(p);
			GlobeAnnotation ga = new GlobeAnnotation(p.toString() , p);
			((RenderableLayer) layer).addRenderable(ga);
		}
		*/
		
		/*
		for (Vec4 intersectionPoint : largeGrid.getIntersectionPoints(child1.getCorners()[5], child2.getCorners()[5])) {
			intersectedCells.addAll(largeGrid.lookupCells(intersectionPoint));
		}
		for (PlanningGrid intersectedCell : intersectedCells) {
			intersectedCell.addCostInterval(new CostInterval(
					"ic",
					ZonedDateTime.now().minusYears(10),
					ZonedDateTime.now().plusYears(10),
					65));
		}
		*/
		
		/*
		Vec4 o = largeGrid.getCorners()[5].toHomogeneousPoint3();
		Cube c = new Cube(o, largeGrid.getUnitAxes(), 100000);
		for (Plane p : c.getPlanes()) {
			System.out.println(p);
		}
		System.out.println("--");
		
		Vec4 ro = c.getRAxis().multiply3(0.25);
		Vec4 so = c.getSAxis().multiply3(0.25);
		Vec4 to = c.getTAxis().multiply3(0.25);
		Vec4 o2 = o.add3(ro).add3(so).add3(to);
		
		Cube c2 = new Cube(o2, c.getUnitAxes(), 50000);
		for (Plane p : c2.getPlanes()) {
			System.out.println(p);
		}
		
		((RenderableLayer) layer).addRenderable(c);
		((RenderableLayer) layer).addRenderable(c2);
		*/
		
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		while (frame.isVisible()) {
			Thread.sleep(1000);
		}
	}

}
