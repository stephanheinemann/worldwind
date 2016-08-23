package com.cfar.swim.worldwind.render;

import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import com.cfar.swim.worldwind.planning.CostInterval;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.GlobeAnnotation;
import gov.nasa.worldwind.render.Material;

public class CostIntervalCylinder extends VerticalCylinder implements TimedRenderable, ThresholdRenderable {

	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	private CostInterval costInterval = new CostInterval("");
	private GlobeAnnotation annotation = null;
	private int thresholdCost = 0;
	private int activeCost = 0;
	
	public CostIntervalCylinder(Position centerPosition, double height, double radius) {
		super(centerPosition, height, radius);
		this.setAttributes(new BasicShapeAttributes());
		this.getAttributes().setInteriorOpacity(0.5);
		this.getAttributes().setEnableLighting(true);
		this.getAttributes().setDrawInterior(true);
		this.getAttributes().setDrawOutline(false);
	}
	
	public CostInterval getCostInterval() {
		return this.costInterval;
	}
	
	public void setCostInterval(CostInterval costInterval) {
		this.costInterval = costInterval;
		this.annotation = new GlobeAnnotation(costInterval.getId(), this.centerPosition);
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
		this.updateAppearance();
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
		if (null != annotation) {
			this.annotation.getAttributes().setVisible(this.activeCost > this.thresholdCost);
		}
	}
	
	// TODO: elements could change color, transparency or even an associated image/icon 
	protected void updateAppearance() {
		this.getAttributes().setInteriorMaterial(new Material(CostColor.getColor(activeCost)));
	}
	
	@Override
	public void render(DrawContext dc) {
		super.render(dc);
		if (null != this.annotation) {
			this.annotation.render(dc);
		}
	}

	public CostIntervalCylinder interpolate(CostIntervalCylinder other) {
		Position center = Position.interpolateGreatCircle(0.5d, this.centerPosition, other.centerPosition);
		double height = (this.getHeight() + other.getHeight()) / 2d;
		double radius = (this.getRadius() + other.getRadius()) / 2d;
		CostIntervalCylinder interpolant = new CostIntervalCylinder(center, height, radius);
		
		Duration startDuration = Duration.between(this.costInterval.getLower(), other.costInterval.getLower());
		startDuration = startDuration.dividedBy(2);
		ZonedDateTime start = this.costInterval.getLower().plus(startDuration);
		
		// ZonedDateTime end = this.costInterval.getUpper();
		this.costInterval.setUpper(start);
		ZonedDateTime end = other.costInterval.getLower();
		
		/*
		Duration endDuration = Duration.between(this.costInterval.getUpper(), other.costInterval.getUpper());
		endDuration = endDuration.dividedBy(2);
		ZonedDateTime end = this.costInterval.getUpper().plus(endDuration);
		*/
		
		// TODO: rounding or conservative ceiling might be more appropriate
		int cost = (this.costInterval.getCost() + other.costInterval.getCost()) / 2;
		
		CostInterval costInterval = new CostInterval(this.costInterval.getId(), start, end, cost);
		interpolant.setCostInterval(costInterval);
		
		return interpolant;
	}
	
	public List<CostIntervalCylinder> interpolate(CostIntervalCylinder other, int steps) {
		List<CostIntervalCylinder> interpolants = new ArrayList<>();
		
		if (1 == steps) {
			interpolants.add(this.interpolate(other));
		} else if (1 < steps) {
			CostIntervalCylinder interpolant = this.interpolate(other);
			interpolants.addAll(this.interpolate(interpolant, steps - 1));
			interpolants.add(interpolant);
			interpolants.addAll(interpolant.interpolate(other, steps - 1));
		}
		
		return interpolants;
	}
	
}
