package com.cfar.swim.worldwind.render;

import java.time.ZoneId;
import java.time.ZonedDateTime;

import com.cfar.swim.worldwind.planning.CostInterval;

import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Path;

public class CostIntervalPath extends Path implements TimedRenderable, ThresholdRenderable {

	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	private CostInterval costInterval = new CostInterval("");
	private int thresholdCost = 0;
	private int activeCost = 0;
	
	public CostIntervalPath(Iterable<? extends Position> positions) {
		super(positions);
		this.setAttributes(new BasicShapeAttributes());
		this.getAttributes().setOutlineOpacity(0.75);
		this.getAttributes().setOutlineWidth(2d);
		this.getAttributes().setEnableLighting(true);
		this.getAttributes().setDrawInterior(false);
		this.getAttributes().setDrawOutline(true);
		this.getAttributes().setOutlineMaterial(Material.PINK);
		
		this.setAltitudeMode(WorldWind.ABSOLUTE);
		this.setPathType(AVKey.GREAT_CIRCLE);
		this.setShowPositions(true);
	}
	
	public CostInterval getCostInterval() {
		return this.costInterval;
	}
	
	public void setCostInterval(CostInterval costInterval) {
		this.costInterval = costInterval;
		this.update();
	}
	
	@Override
	public void setThreshold(int threshold) {
		this.thresholdCost = threshold;
		this.updateVisibility();
	}

	@Override
	public int getThreshold() {
		return this.thresholdCost;
	}

	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}
	
	protected void update() {
		this.updateActiveCost();
		this.updateVisibility();
		//this.updateAppearance();
	}
	
	protected void updateActiveCost() {
		if (this.costInterval.contains(this.time)) {
			this.activeCost = this.costInterval.getWeightedCost();
		} else {
			this.activeCost = 0;
		}
	}
	
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}
	
	// TODO: elements could change color, transparency or even an associated image/icon 
	protected void updateAppearance() {
		this.getAttributes().setOutlineMaterial(new Material(CostColor.getColor(activeCost)));
	}

}
