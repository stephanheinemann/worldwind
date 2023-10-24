package com.cfar.swim.worldwind.planners.rl;

import java.util.Map;
import java.util.Random;

import com.cfar.swim.worldwind.planners.rl.qlearning.Action;
import com.cfar.swim.worldwind.planners.rl.qlearning.RLWaypoint;

/**
 * Realizes a transition object to store a state, action, next state and reward tuple
 * 
 * @author Rafaela Seguro
 *
 */

public class Snapshot {
	
	/** the state*/     
	private State state = null;
	
	/** the reward */
	private double reward = 0;
	
	/** indicates if it reached the goal */
	private boolean done = false;
	
	/** indicates if it failed */
	private boolean failure = false;
	
	
	/** Constructs a snapshot, which stores the new information about the environment after a step
	 * 
	 * @param the state
	 * @param the reward
	 * @param the done boolean
	 * @param the failure boolean
	 */
	public Snapshot(State state, double reward, boolean done, boolean failure) {
		this.state = state;
		this.reward = reward;
		this.done = done;
		this.failure = failure;
	}
	
	/**
	 * Gets the state stored in this transition.
	 * 
	 * @return the state
	 */
	public State getState() {
		return this.state;
	}
	
	/**
	 * Gets the reward stored in this transition.
	 * 
	 * @return the reward
	 */
	public double getReward() {
		return this.reward;
	}
	
	/**
	 * Indicates if with this step the agent reached the goal.
	 * 
	 * @return true if it has reached the goal
	 */
	public boolean isDone() {
		return this.done;
	}
	
	/**
	 * Indicates if with this step the agent failed.
	 * 
	 * @return true if it has failed
	 */
	public boolean failed() {
		return this.failure;
	}

}
