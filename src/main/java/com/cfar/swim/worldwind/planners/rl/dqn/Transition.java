package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.Map;
import java.util.Random;

import com.cfar.swim.worldwind.planners.rl.Action;
import com.cfar.swim.worldwind.planners.rl.RLWaypoint;
import com.cfar.swim.worldwind.planners.rl.State;

/**
 * Realizes a transition object to store a state, action, next state and reward tuple
 * 
 * @author Rafaela Seguro
 *
 */

public class Transition {
	
	/** the state*/     
	private float[] state = new float[State.ID_SIZE];
	
	/** the action */
	private int action = 0;
	
	/** the next state */
	private float[] nextState =  new float[State.ID_SIZE];
	
	/** the reward */
	private double reward = 0;
	
	/** indicates if it is a terminal state or not */
	private boolean done = false;
	
	
	/** Constructs a transition tuple
	 * 
	 * @param the state
	 * @param the action
	 * @param the next state
	 * @param the reward
	 */
	public Transition(float[] state, int action, float[] nextState, double reward, boolean done) {
		this.state = state;
		this.action = action;
		this.nextState = nextState;
		this.reward = reward;
		this.done = done;
	}
	
	/**
	 * Gets the state stored in this transition.
	 * 
	 * @return the state
	 */
	public float[] getState() {
		return this.state;
	}
	
	/**
	 * Gets the action stored in this transition.
	 * 
	 * @return the action
	 */
	public int getAction() {
		return this.action;
	}
	
	/**
	 * Gets the next state stored in this transition.
	 * 
	 * @return the next state
	 */
	public float[] getNextState() {
		return this.nextState;
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
	 * Indicates if with this transition the agent reached the goal.
	 * 
	 * @return true if it has reached the goal
	 */
	public boolean isDone() {
		return this.done;
	}

}
