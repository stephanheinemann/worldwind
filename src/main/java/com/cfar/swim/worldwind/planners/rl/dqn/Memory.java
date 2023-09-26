package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.Arrays;
import java.util.Random;

import com.cfar.swim.worldwind.planners.rl.Action;
import com.cfar.swim.worldwind.planners.rl.State;

import ai.djl.ndarray.NDManager;

/**
 * Realizes a replay memory object to store transitions and to be used by the Deep Neural Network
 * 
 * @author Rafaela Seguro
 *
 */

public class Memory {
	
	/** random number */
	private final Random rand = new Random();
	
	/** the memory's capacity */
	private int capacity = 0;
	
	/** the array to store transitions*/
	private Transition[] memory;
	
	/** stores the previous state's ID*/
	private float[] state_prev;
	
	/** stores the most recent chosen action's index*/
	private int action;
	
	/** stores the most recent received reward*/
	private double reward;
	
	/** indicates if agent has reached the goal, true if yes*/
	private boolean done;
	
	/** keeps track of the last added transition position*/
	private int head;
	
	/** keeps track of how many memory positions are occupied*/
	private int size;
	
	
	/** Constructs a replay memory object
	 * 
	 * @param the memory's capacity
	 */
	public Memory(int capacity) {
		this.capacity = capacity;
		this.memory = new Transition[capacity];
		
		reset();
	}
	
	/**
	 * Sets the current state and adds the new transition to the memory.
	 * 
	 * @param the current state
	 */
	public void setState(float[] state) {
		if (state_prev != null)
			add(new Transition(state_prev, action, state, reward, done));
		state_prev = state;
	}
	
	/**
	 * Sets the most recent chosen action.
	 * 
	 * @param the action
	 */
	public void setAction(int action) {
		this.action = action;
	}
	
	/**
	 * Sets the most recent received reward.
	 * 
	 * @param the reward
	 */
	public void setReward(double reward, boolean done) {
		this.reward = reward;
		this.done = done;
		
		if(done) {
			add(new Transition (state_prev, action, null, reward, done));
			state_prev = null;
			action = -1;
		}
	}
	
	/**
	 * Gets the memory's current size
	 * 
	 * @return the memory's size
	 */
	public int getSize() {
		return size;
	}
	
	/**
	 * Resets the replay memory.
	 */
	public void reset() {
		state_prev = null;
		action = -1;
		reward = 0;
		done = false;
		head = -1;
		size = 0;
	}
	
	/**
	 * Adds a transition to the replay memory.
	 * 
	 * @param the transition
	 */
	public void add(Transition transition) {
		head += 1;
		// If memory is full it goes back to the beginning and starts replacing older transitions
		if (head >= capacity)
			head = 0;
		
		memory[head] = transition;
		
		if (size < capacity)
			size++;
	}
	
	/**
	 * Samples a chunk of transitions from the memory with the size indicated
	 * 
	 * @param the size of the sample
	 * 
	 * @return the sample
	 */
	public Transition[] sample(int sampleSize) {
		Transition[] sample = new Transition[sampleSize];
		for (int i = 0; i<sampleSize; i++) {
			sample[i] = memory[rand.nextInt(size)];
		}
		return sample;
	}
	
	/**
	 * Creates a batch of training data from a set of transitions
	 * 
	 * @param the array of transitions
	 * @param the manager for NDArray operations
	 * @param the size of the batch
	 * 
	 * @return the memory batch
	 */
	public MemoryBatch getBatch(Transition[] transitions, NDManager manager, int batchSize) {
		
		// Initiate arrays to store batch data
		float[][] states = new float[batchSize][];
		int[] actions = new int[batchSize];
		float[][] nextStates = new float[batchSize][];
		double[] rewards = new double[batchSize];
		boolean [] dones = new boolean[batchSize];
		
		int index = head;
		// Fills arrays with information from the circular buffer of transitions
		for (int i = 0; i< batchSize; i++) {
			index++;
			// If it reaches the end of the buffer, it goes back to the beggining since it is circular
			if (index >= batchSize)
				index = 0;
			states[i] = transitions[index].getState();
			actions[i] = transitions[index].getAction();
			// If next state is null (transition where episode ended), create array of zeros
			float[] nextState = transitions[index].getNextState();
			nextStates[i] = nextState != null ? nextState : new float[states[i].length];
			rewards[i] = transitions[index].getReward();
			dones[i] = transitions[index].isDone();
		}
		
		return new MemoryBatch(manager.create(states), manager.create(actions), manager.create(nextStates), manager.create(rewards), manager.create(dones));
	}
	
	
	/**
	 * Samples a random memory batch
	 * 
	 * @param the sample size
	 * @param the manager for NDArray operations
	 * 
	 * @return the memory batch
	 */
	public MemoryBatch sampleBatch(int sampleSize, NDManager manager) {
		return getBatch(sample(sampleSize), manager, sampleSize);
	}
	
	
	/**
	 * Gets all the transitions in memory in batch format
	 * 
	 * @param the manager for NDArray operations
	 * 
	 * @return the memory batch
	 */
	public MemoryBatch getOrderedBatch(NDManager manager) {
		return getBatch(memory, manager, this.size);
	}
	
	/**
	 * Gets a specific transition by memory index
	 * 
	 * @param the index
	 * 
	 * @return the transition
	 */
	public Transition getTransition(int index) {
		
		if (index < 0 || index >= size)
			throw new ArrayIndexOutOfBoundsException("Index out of bound " + index);
		
		return memory[index];
	}
	

}
