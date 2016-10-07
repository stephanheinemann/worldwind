package com.cfar.swim.worldwind.planning;

import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;

// TODO: a planning roadmap could be fed by AIXM data to establish valid VFR/IFR routes
// TODO: there could be a geometric roadmap base class (similar to planning grid)
public class PlanningRoadmap implements Environment {

	@Override
	public ZonedDateTime getTime() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setTime(ZonedDateTime time) {
		// TODO Auto-generated method stub

	}

	@Override
	public void render(DrawContext dc) {
		// TODO Auto-generated method stub

	}

	@Override
	public void setThreshold(double threshold) {
		// TODO Auto-generated method stub

	}

	@Override
	public double getThreshold() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public Globe getGlobe() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setGlobe(Globe globe) {
		// TODO Auto-generated method stub

	}

	@Override
	public Position getCenterPosition() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Set<? extends Environment> getNeighbors() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean areNeighbors(Environment neighbor) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Set<Position> getNeighbors(Position position) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean areNeighbors(Position position, Position neighbor) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Set<? extends Environment> getChildren() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double getDistance(Position position1, Position position2) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void addCostInterval(CostInterval costInterval) {
		// TODO Auto-generated method stub

	}

	@Override
	public void removeCostInterval(CostInterval costInterval) {
		// TODO Auto-generated method stub

	}

	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double getCost(ZonedDateTime start, ZonedDateTime end) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getStepCost(Position position, Position neighbor, ZonedDateTime start, ZonedDateTime end,
			CostPolicy policy) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getStepCost(Environment neighbor, ZonedDateTime start, ZonedDateTime end, CostPolicy policy) {
		// TODO Auto-generated method stub
		return 0;
	}

}
