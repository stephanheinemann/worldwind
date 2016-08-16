package com.cfar.swim.worldwind.javafx;

import java.util.List;

import com.cfar.swim.worldwind.render.ThresholdRenderable;

import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.Renderable;
import javafx.event.EventHandler;
import javafx.scene.control.Slider;
import javafx.scene.input.MouseEvent;

public class ThresholdCostSlider extends Slider {

	private class ThresholdCostHandler implements EventHandler<MouseEvent> {

		ThresholdCostSlider slider = null;

		public ThresholdCostHandler(ThresholdCostSlider slider) {
			this.slider = slider;
		}

		@Override
		public void handle(MouseEvent event) {
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

	private WorldWindow worldWindow;

	public ThresholdCostSlider(WorldWindow worldWindow) {
		super(0, 100, 0);
		this.worldWindow = worldWindow;
		this.setOnMouseReleased(new ThresholdCostHandler(this));
	}

}
