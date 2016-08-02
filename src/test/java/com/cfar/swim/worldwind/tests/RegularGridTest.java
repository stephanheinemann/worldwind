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

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;

import javax.xml.bind.JAXBException;

import org.junit.Test;
import org.xml.sax.InputSource;

import com.cfar.swim.worldwind.iwxxm.IwxxmUpdater;
import com.cfar.swim.worldwind.planning.NonUniformCostIntervalGrid;

import gov.nasa.worldwind.BasicModel;
import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.awt.WorldWindowGLCanvas;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.layers.LayerList;
import gov.nasa.worldwind.layers.RenderableLayer;

public class RegularGridTest {
	
	static Model model;
	static NonUniformCostIntervalGrid uvicGrid;
	
	// TODO: proper initialization with a sequence of tests
	private static class AppFrame extends javax.swing.JFrame {
        
		private static final long serialVersionUID = 1L;

		public AppFrame() {
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
            uvicGrid = new NonUniformCostIntervalGrid(uvicBox);
            uvicGrid.setThresholdCost(0);
            uvicGrid.addChildren(uvicBox.getTLength() / 5.0);
            //uvicGrid.setThresholdCost(50);
            RenderableLayer renderableLayer = new RenderableLayer();
            renderableLayer.addRenderable(uvicGrid);
            LayerList layers = wwd.getModel().getLayers();
            layers.add(renderableLayer);
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
		
		IwxxmUpdater iwxxmUpdater = new IwxxmUpdater(model, uvicGrid);
		iwxxmUpdater.add(new InputSource(new FileInputStream("src/test/resources/xml/iwxxm/sigmet-A6-1a-TS.xml")));
	
		while (frame.isVisible()) {
			Thread.sleep(1000);
		}
	}

}
