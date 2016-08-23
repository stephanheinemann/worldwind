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
package com.cfar.swim.worldwind.javafx;

import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.List;

import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.Renderable;
import javafx.event.EventHandler;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.util.Callback;
import jfxtras.scene.control.LocalDateTimePicker;

/**
 * Realizes a planning time picker widget to input the current planning time
 * of a world window.
 * 
 * @see WorldWindow
 * @see LocalDatTimePicker
 * 
 * @author Stephan Heinemann
 *
 */
public class PlanningTimePicker extends LocalDateTimePicker {
	
	/**
	 * Realizes a planning time callback that updates all timed renderables of
	 * a world window.
	 * 
	 * @see TimedRenderable
	 */
	private class PlanningTimeCallback implements Callback<LocalDateTime, Boolean> {

		/** the planning time picker widget */
		private PlanningTimePicker picker = null;

		/**
		 * Constructs a planning time callback with a specified planning time
		 * picker.
		 * 
		 * @param picker the planning time picker
		 */
		public PlanningTimeCallback(PlanningTimePicker picker) {
			this.picker = picker;
		}

		/**
		 * Updates the time of all timed renderables of a world window.
		 */
		@Override
		public Boolean call(LocalDateTime localDateTime) {
			if (null != localDateTime) {
				List<Layer> renderableLayers = this.picker.worldWindow.getModel().getLayers().getLayersByClass(RenderableLayer.class);
				for (Layer layer : renderableLayers) {
					for (Renderable renderable : ((RenderableLayer) layer).getRenderables()) {
						if (renderable instanceof TimedRenderable) {
							((TimedRenderable) renderable).setTime(ZonedDateTime.of(localDateTime, ZoneId.of("UTC")));
						}
					}
				}
				this.picker.worldWindow.redrawNow();
			}
			return true;
		}

	}
	
	/**
	 * Realizes a planning time key handler to change the planning time of a
	 * world window.
	 */
	private class PlanningTimeKeyHandler implements EventHandler<KeyEvent> {

		/** the maximum planning time step duration */
		private final Duration DURATION_MAX = Duration.ofHours(24);
		
		/** the planning time picker */
		private PlanningTimePicker picker = null;
		
		/** the current planning time step duration */
		private Duration duration = Duration.ofMinutes(10);
		
		/**
		 * Constructs a planning time key handler with a specified planning
		 * time picker.
		 * 
		 * @param picker the planning time picker
		 */
		public PlanningTimeKeyHandler(PlanningTimePicker picker) {
			this.picker = picker;
		}
		
		/**
		 * Handles the key events to change the current planning time and
		 * planning time step duration.
		 */
		@Override
		public void handle(KeyEvent event) {
			if (KeyCode.RIGHT.equals(event.getCode())) {
				if (event.isControlDown()) {
					if (0 > this.duration.compareTo(DURATION_MAX)) {
						this.duration = this.duration.plusMinutes(1);
					}
				} else {
					this.picker.setLocalDateTime(this.picker.getLocalDateTime().plus(this.duration));
					this.picker.getValueValidationCallback().call(this.picker.getLocalDateTime());
				}
				event.consume();
			} else if (KeyCode.LEFT.equals(event.getCode())) {
				if (event.isControlDown()) {
					if (0 < this.duration.compareTo(Duration.ZERO)) {
						this.duration = this.duration.minusMinutes(1);
					}
				} else {
					this.picker.setLocalDateTime(this.picker.getLocalDateTime().minus(this.duration));
					this.picker.getValueValidationCallback().call(this.picker.getLocalDateTime());
				}
				event.consume();
			}
		}
		
	}

	/** the world window of this planning time picker */
	private WorldWindow worldWindow = null;

	/**
	 * Constructs a planning time picker with a specified world window.
	 * 
	 * @param worldWindow the world window the planning time is applied to
	 */
	public PlanningTimePicker(WorldWindow worldWindow) {
		super(LocalDateTime.now(ZoneId.of("UTC")));
		this.worldWindow = worldWindow;
		this.setValueValidationCallback(new PlanningTimeCallback(this));
		//this.setOnKeyPressed(new PlanningTimeKeyHandler(this));
		this.addEventFilter(KeyEvent.KEY_PRESSED, new PlanningTimeKeyHandler(this));
	}

}
