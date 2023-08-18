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
	
	/** stores the current state's ID*/
	private int state;
	
	/** stores the most recent chosen action's ID*/
	private int action;
	
	/** stores the most recent received reward*/
	private float reward;
	
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
	public void setState(int state) {
		if (this.state != -1)
			add(new Transition(this.state, action, state, reward));
		this.state = state;
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
	public void setReward(float reward, boolean done) {
		this.reward = reward;
		this.done = done;
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
		this.state = -1;
		this.action = -1;
		this.reward = 0;
		this.done = false;
		this.head = -1;
		this.size = 0;
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
	 * Converts an array of transitions to a MemoryBatch, with the given size and manager
	 * 
	 * @param the array of transitions
	 * @param the NDArray manager
	 * @param the size of the batch
	 * 
	 * @return the memory batch
	 */
	public MemoryBatch getBatch(Transition[] transitions, NDManager manager, int batchSize) {
		
		// Initiate arrays that will be then converted to NDArrays
		int[] states = new int[batchSize];
		int[] actions = new int[batchSize];
		int[] nextStates = new int[batchSize];
		float[] rewards = new float[batchSize];
		boolean [] dones = new boolean[batchSize];
		
		int index = head;
		// Fills arrays with information from the array of transitions
		for (int i = 0; i< batchSize; i++) {
			index++;
			// Can't understand why, they are in random order in the array of transitions
			if (index >= batchSize)
				index = 0;
			// Nao sei se vai funcionar com os IDs mas nao sei criar um NDArray de states
			states[i] = transitions[index].getState();
			actions[i] = transitions[index].getAction();
			// Eles fazem um check com o next state que eu nao percebo
			nextStates[i] = transitions[index].getNextState();
			rewards[i] = transitions[index].getReward();
			dones[i] = transitions[index].isDone();
		}
		
		return new MemoryBatch(manager.create(states), manager.create(actions), manager.create(nextStates), manager.create(rewards), manager.create(dones));
	}
	
	
	/**
	 * Samples a random memory batch
	 * 
	 * @param the sample size
	 * @param the batch manager
	 * 
	 * @return the memory batch
	 */
	public MemoryBatch sampleBatch(int sampleSize, NDManager manager) {
		return getBatch(sample(sampleSize), manager, sampleSize);
	}
	
	
	/**
	 * Gets all the transitions in memory in batch format
	 * 
	 * @param the batch manager
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
