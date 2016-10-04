/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
import java.util.Set;

import javax.xml.bind.JAXBException;

import org.junit.Test;
import org.xml.sax.InputSource;

import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.iwxxm.IwxxmUpdater;
import com.cfar.swim.worldwind.javafx.PlanningTimePicker;
import com.cfar.swim.worldwind.javafx.SwimDataListView;
import com.cfar.swim.worldwind.javafx.ThresholdCostSlider;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.NonUniformCostIntervalGrid;
import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;

import gov.nasa.worldwind.BasicModel;
import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.awt.WorldWindowGLCanvas;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.geom.Sphere;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.LayerList;
import gov.nasa.worldwind.layers.RenderableLayer;
import javafx.embed.swing.JFXPanel;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.paint.Color;

public class RegularGridTest {
	
	static Model model;
	static NonUniformCostIntervalGrid uvicGrid;
	static NonUniformCostIntervalGrid largeGrid;
	static NonUniformCostIntervalGrid tsGrid;
	static NonUniformCostIntervalGrid tcGrid;
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
        
            // TODO: separate test procedures
            Sector uvic = new Sector(
            	Angle.fromDegrees(48.462836),
            	Angle.fromDegrees(48.463418),
            	Angle.fromDegrees(-123.312186),
        		Angle.fromDegrees(-123.310799)
            	);
            gov.nasa.worldwind.geom.Box uvicBox = Sector.computeBoundingBox(wwd.getModel().getGlobe(), 1.0, uvic, 0.0, 150.0);
            uvicGrid = new NonUniformCostIntervalGrid(new com.cfar.swim.worldwind.geom.Box(uvicBox));
            uvicGrid.setThreshold(0);
            uvicGrid.addChildren(uvicBox.getTLength() / 5.0);
            //uvicGrid.setThresholdCost(50);
            //uvicGrid.setGlobe(model.getGlobe());
            
            RenderableLayer renderableLayer = new RenderableLayer();
            renderableLayer.addRenderable(uvicGrid);
            LayerList layers = wwd.getModel().getLayers();
            layers.add(renderableLayer);
            
            
            Sector large = new Sector(
            	Angle.fromDegrees(0.0),
            	Angle.fromDegrees(90.0),
            	Angle.fromDegrees(0.0),
            	Angle.fromDegrees(90.0));
            gov.nasa.worldwind.geom.Box largeBox = Sector.computeBoundingBox(wwd.getModel().getGlobe(), 1.0, large, 0.0, 500000.0);
            largeGrid = new NonUniformCostIntervalGrid(new com.cfar.swim.worldwind.geom.Box(largeBox));
            largeGrid.setThreshold(0);
            largeGrid.addChildren(largeBox.getTLength() / 4.0);
            largeGrid.addChildren(0, 8, 3, 2, 2, 2);
            largeGrid.addChildren(1, 9, 3, 2, 2, 2);
            largeGrid.addChildren(0, 9, 1, 2, 2, 2);
            largeGrid.addChildren(2, 9, 1, 2, 2, 2);
            //largeGrid.setGlobe(model.getGlobe());
            //renderableLayer.addRenderable(largeGrid);
            
            Sector ts = new Sector(
                	Angle.fromDegrees(50.0),
                	Angle.fromDegrees(60.0),
                	Angle.fromDegrees(-15.0),
                	Angle.fromDegrees(5.0));
            gov.nasa.worldwind.geom.Box tsBox = Sector.computeBoundingBox(wwd.getModel().getGlobe(), 1.0, ts, 0.0, 500000.0);
            tsGrid = new NonUniformCostIntervalGrid(new com.cfar.swim.worldwind.geom.Box(tsBox));
            tsGrid.setThreshold(0);
            tsGrid.addChildren(tsGrid.getTLength() / 4.0);
            tsGrid.addChildren(3, 3, 0, 2, 2, 2);
            renderableLayer.addRenderable(tsGrid);
            
            Sector tc = new Sector(
                	Angle.fromDegrees(25.0),
                	Angle.fromDegrees(30.0),
                	Angle.fromDegrees(-75.0),
                	Angle.fromDegrees(-70.0));
            gov.nasa.worldwind.geom.Box tcBox = Sector.computeBoundingBox(wwd.getModel().getGlobe(), 1.0, tc, 0.0, 50000.0);
            tcGrid = new NonUniformCostIntervalGrid(new com.cfar.swim.worldwind.geom.Box(tcBox));
            tcGrid.setThreshold(0);
            tcGrid.addChildren(tcGrid.getTLength() / 4.0);
            renderableLayer.addRenderable(tcGrid);
            
            iris = new Iris(new Position(ts.getCentroid(), 50000), 5000, CombatIdentification.FRIEND);
            iris.setCostInterval(new CostInterval(
            				"Iris",
            				ZonedDateTime.now(ZoneId.of("UTC")).minusYears(10),
            				ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
            				70));
            iris.getDepiction().setDesignation("Iris");
            renderableLayer.addRenderable(iris);
            
            // TODO: add time slider with steps between min and max time (set using calender-like input)
            this.timePanel = new JFXPanel();
        	this.timePanel.setSize(300, 400);
        	this.timePanel.setScene(createScene(wwd));
        	this.getContentPane().add(this.timePanel, BorderLayout.WEST);
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
		largeGrid.setGlobe(model.getGlobe());
		//IwxxmUpdater largeUpdater = new IwxxmUpdater(model, largeGrid);
		//largeUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1a-TS2.xml")));
		
		tsGrid.setGlobe(model.getGlobe());
		IwxxmUpdater tsUpdater = new IwxxmUpdater(model, tsGrid);
		tsUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1a-TS.xml")));
		tsUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1b-TS.xml")));
		tsUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1b-CNL.xml")));
		
		tcGrid.setGlobe(model.getGlobe());
		/*
		IwxxmUpdater tcUpdater = new IwxxmUpdater(model, tcGrid);
		tcUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-2-TC.xml")));
		*/
		
		//list.registerDataActivationListerner(largeUpdater);
		list.registerDataActivationListerner(tsUpdater);
		//list.registerDataActivationListerner(tcUpdater);
		
		// TODO: this seems to be the expected rst versus xyz bug
		System.out.println("iris location " + iris.getLocation());
		System.out.println("checking position " + iris.getReferencePosition());
		Vec4 point = tsGrid.getGlobe().computePointFromPosition(iris.getReferencePosition());
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
		Layer layer = model.getLayers().getLayersByClass(RenderableLayer.class).get(0);
    	((RenderableLayer) layer).addRenderable(sphere);
		
		Set<? extends NonUniformCostIntervalGrid> cells = tsGrid.lookupCells(iris.getReferencePosition());
		System.out.println("found iris cells " + cells.size());
		
		//Set<Position> neighbors = tsGrid.getNeighbors(iris.getReferencePosition());
		//Set<Position> neighbors = tsGrid.getNeighbors(model.getGlobe().computePositionFromPoint(tsGrid.getChild(2, 2, 2).getBottomCenter()));
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
        
        NonUniformCostIntervalGrid irisCell = tsGrid.lookupCells(iris.getReferencePosition()).iterator().next();
        Set<? extends RegularGrid> irisNeighbors = irisCell.getNeighbors();
        for (RegularGrid irisNeighbor : irisNeighbors) {
        	((NonUniformCostIntervalGrid) irisNeighbor).addCostInterval(new CostInterval(
    				"neighbor cell",
    				ZonedDateTime.now(ZoneId.of("UTC")).minusYears(10),
    				ZonedDateTime.now(ZoneId.of("UTC")).plusYears(10),
    				50));
        }
        
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		while (frame.isVisible()) {
			Thread.sleep(1000);
		}
	}

}
