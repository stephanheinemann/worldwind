package com.cfar.swim.worldwind.planners.rl;

/**
 * Realizes a transition object to store a state, action, next state and reward tuple
 * 
 * @author Rafaela Seguro
 *
 */

public class Transition {
	
	/** the state*/     
	private float[] state = new float[State.ID_SIZE];
	
	/** the next state */
	private float[] nextState =  new float[State.ID_SIZE];

	/** the action */
	private int action = 0;
	
	/** the reward */
	private double reward = 0;
	
	/** indicates if it is a terminal state or not */
	private boolean done = false;
	
	/** stores the transition's index in memory */
	private int index = 0;
	
	
	
	/** Constructs a transition tuple
	 * 
	 * @param the state
	 * @param the action
	 * @param the next state
	 * @param the reward
	 */
	public Transition(float[] state, float[] nextState, int action, double reward, boolean done) {
		this.state = state;
		this.nextState = nextState;
		this.action = action;
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
	 * Gets the next state stored in this transition.
	 * 
	 * @return the next state
	 */
	public float[] getNextState() {
		return this.nextState;
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
	
	/**
	 * Sets the transition's index in memory.
	 * 
	 * @param the index
	 */
	public void setIndex(int index) {
		this.index = index;
	}
	
	/**
	 * Gets the transition's index in memory.
	 * 
	 * @return the index
	 */
	public int getIndex() {
		return this.index;
	}

}
