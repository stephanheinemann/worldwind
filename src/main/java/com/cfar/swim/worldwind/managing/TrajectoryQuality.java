package com.cfar.swim.worldwind.managing;

import com.cfar.swim.worldwind.planning.Trajectory;

/**
 * Realizes a trajectory quality.
 * 
 * @author Stephan Heinemann
 *
 * @see Quality
 */
public class TrajectoryQuality implements Quality {
	
	/** the quality of this trajectory quality */
	final Trajectory quality;
	
	/**
	 * Constructs a new trajectory quality based on a trajectory.
	 * 
	 * @param quality the trajectory
	 * 
	 * @throws IllegalArgumentException if the quality is invalid
	 */
	public TrajectoryQuality(Trajectory quality) {
		if (null == quality) {
			throw new IllegalArgumentException("quality is invalid");
		}
		
		this.quality = quality;
	}
	
	/**
	 * Gets the trajectory quality in overall cost.
	 * 
	 * @return the trajectory quality in overall cost
	 * 
	 * @see Quality#get()
	 */
	@Override
	public double get() {
		double measure = 0d;
		
		if (!quality.isEmpty()) {
			measure = 1d / quality.getLastWaypoint().getCost();
			// TODO: weighted quality
			// TODO: number of legs / edges (heading changes)
			// TODO: horizontal versus vertical distance (cost calculation)
			// TODO: ETE, ETA
		}
		
		return measure;
	}

}
