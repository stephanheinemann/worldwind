package com.cfar.swim.worldwind.javafx;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.List;

import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.Renderable;
import javafx.util.Callback;
import jfxtras.scene.control.LocalDateTimePicker;

public class PlanningTimePicker extends LocalDateTimePicker {

	private class PlanningTimeCallback implements Callback<LocalDateTime, Boolean> {

		PlanningTimePicker picker = null;

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

	private WorldWindow worldWindow = null;

	public PlanningTimePicker(WorldWindow worldWindow) {
		super(LocalDateTime.now(ZoneId.of("UTC")));
		this.worldWindow = worldWindow;
		this.setValueValidationCallback(new PlanningTimeCallback(this));
	}

}
