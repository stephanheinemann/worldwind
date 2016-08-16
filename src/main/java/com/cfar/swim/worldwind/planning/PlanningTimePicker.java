package com.cfar.swim.worldwind.planning;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;

import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.WorldWindow;
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
				this.picker.timedRenderable.setTime(ZonedDateTime.of(localDateTime, ZoneId.of("UTC")));
				this.picker.worldWindow.redrawNow();
			}
			return true;
		}

	};

	private TimedRenderable timedRenderable = null;
	private WorldWindow worldWindow = null;

	public PlanningTimePicker(TimedRenderable timedRenderable, WorldWindow worldWindow) {
		super(LocalDateTime.now(ZoneId.of("UTC")));
		this.timedRenderable = timedRenderable;
		this.worldWindow = worldWindow;
		this.setValueValidationCallback(new PlanningTimeCallback(this));
	}

}
