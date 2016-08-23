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

public class PlanningTimePicker extends LocalDateTimePicker {

	private class PlanningTimeCallback implements Callback<LocalDateTime, Boolean> {

		private PlanningTimePicker picker = null;

		public PlanningTimeCallback(PlanningTimePicker picker) {
			this.picker = picker;
		}

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

	};
	
	private class PlanningTimeKeyHandler implements EventHandler<KeyEvent> {

		private final Duration DURATION_MAX = Duration.ofHours(24);
		
		private PlanningTimePicker picker = null;
		private Duration duration = Duration.ofMinutes(10);
		
		public PlanningTimeKeyHandler(PlanningTimePicker picker) {
			this.picker = picker;
		}
		
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

	private WorldWindow worldWindow = null;

	public PlanningTimePicker(WorldWindow worldWindow) {
		super(LocalDateTime.now(ZoneId.of("UTC")));
		this.worldWindow = worldWindow;
		this.setValueValidationCallback(new PlanningTimeCallback(this));
		//this.setOnKeyPressed(new PlanningTimeKeyHandler(this));
		this.addEventFilter(KeyEvent.KEY_PRESSED, new PlanningTimeKeyHandler(this));
	}

}
