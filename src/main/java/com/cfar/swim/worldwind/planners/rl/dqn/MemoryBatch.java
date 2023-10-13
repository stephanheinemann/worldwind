package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.Map;
import java.util.Random;

import com.cfar.swim.worldwind.planners.rl.Action;
import com.cfar.swim.worldwind.planners.rl.RLWaypoint;
import com.cfar.swim.worldwind.planners.rl.State;

import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDList;

/**
 * Realizes a memory batch to be used in the training of the Deep Neural Network
 * 
 * @author Rafaela Seguro
 *
 */

public class MemoryBatch extends NDList{
	
	/** Constructs a memory batch
	 * 
	 * @param arrays
	 */
	public MemoryBatch(NDArray... arrays) {
		super(arrays);
	}
	
	/**
	 * Gets the states.
	 * 
	 * @return the states
	 */
	public NDArray getStates() {
		return get(0);
	}
	
	/**
	 * Gets the next states.
	 * 
	 * @return the next states
	 */
	public NDArray getNextStates() {
		return get(1);
	}
	
	/**
	 * Gets the actions.
	 * 
	 * @return the actions
	 */
	public NDArray getActions() {
		return get(2);
	}
	
	/**
	 * Gets the rewards.
	 * 
	 * @return the rewards
	 */
	public NDArray getRewards() {
		return get(3);
	}
	
	/**
	 * Gets the booleans that indicate if it reached the goal or not.
	 * 
	 * @return the booleans
	 */
	public NDArray getDones() {
		return get(4);
	}

}