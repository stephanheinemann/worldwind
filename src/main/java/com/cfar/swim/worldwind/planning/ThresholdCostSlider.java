package com.cfar.swim.worldwind.planning;

import javafx.scene.control.Slider;

import com.cfar.swim.worldwind.render.ThresholdRenderable;

import gov.nasa.worldwind.WorldWindow;
import javafx.event.EventHandler;
import javafx.scene.input.MouseEvent;

public class ThresholdCostSlider extends Slider {

	private class ThresholdCostHandler implements EventHandler<MouseEvent> {

		ThresholdCostSlider slider = null;

		public ThresholdCostHandler(ThresholdCostSlider slider) {
			this.slider = slider;
		}

		@Override
		public void handle(MouseEvent event) {
			this.slider.thresholdRenderable.setThreshold((int) slider.getValue());
			this.slider.worldWindow.redrawNow();
		}

	}

	private ThresholdRenderable thresholdRenderable;
	private WorldWindow worldWindow;

	public ThresholdCostSlider(ThresholdRenderable thresholdRenderable, WorldWindow worldWindow) {
		super(0, 100, 0);
		this.thresholdRenderable = thresholdRenderable;
		this.worldWindow = worldWindow;
		this.setOnMouseReleased(new ThresholdCostHandler(this));
	}

}
