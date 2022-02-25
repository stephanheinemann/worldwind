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
package com.cfar.swim.worldwind.javafx;

import java.util.List;

import com.cfar.swim.worldwind.render.ThresholdRenderable;

import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.Renderable;
import javafx.event.EventHandler;
import javafx.scene.control.Slider;
import javafx.scene.input.InputEvent;

/**
 * Realizes a threshold cost slider to input the current threshold cost of a
 * world window.
 * 
 * @see WorldWindow
 * @see Slider
 * 
 * @author Stephan Heinemann
 *
 */
public class ThresholdCostSlider extends Slider {

	/**
	 * Realizes a threshold cost input handler to change the current threshold cost
	 * of a world window.
	 * 
	 * @see ThresholdRenderable
	 */
	private class ThresholdCostInputHandler implements EventHandler<InputEvent> {

		/** the threshold cost slider */
		ThresholdCostSlider slider = null;

		/**
		 * Constructs a threshold cost input handler with a specified threshold
		 * cost slider.
		 * 
		 * @param slider the threshold cost slider
		 */
		public ThresholdCostInputHandler(ThresholdCostSlider slider) {
			this.slider = slider;
		}

		/**
		 * Handles the input events to change the current threshold cost of
		 * threshold renderables of a world window.
		 */
		@Override
		public void handle(InputEvent event) {
			List<Layer> renderableLayers = this.slider.worldWindow.getModel().getLayers().getLayersByClass(RenderableLayer.class);
			for (Layer layer : renderableLayers) {
				for (Renderable renderable : ((RenderableLayer) layer).getRenderables()) {
					if (renderable instanceof ThresholdRenderable) {
						((ThresholdRenderable) renderable).setThreshold((int) this.slider.getValue());
					}
				}
			}
			this.slider.worldWindow.redrawNow();
		}

	}

	/** the world window of this threshold cost slider */
	private WorldWindow worldWindow = null;

	/**
	 * Constructs a threshold cost slider with a specified world window.
	 * 
	 * @param worldWindow the world window the threshold cost is applied to
	 */
	public ThresholdCostSlider(WorldWindow worldWindow) {
		super(-100, 100, 0);
		this.worldWindow = worldWindow;
		this.setOnMouseReleased(new ThresholdCostInputHandler(this));
		this.setOnKeyPressed(new ThresholdCostInputHandler(this));
	}

}
