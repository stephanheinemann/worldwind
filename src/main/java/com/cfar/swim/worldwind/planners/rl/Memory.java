package com.cfar.swim.worldwind.planners.rl;

import java.util.Random;

import ai.djl.ndarray.NDManager;

/**
 * Realizes a replay memory object to store transitions and to be used by the Deep Q-Network
 * 
 * @author Rafaela Seguro
 *
 */

public class Memory {
	
	/** random number */
	private final Random rand = new Random(0);
	
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
	
	/** indicates if agent has failed, true if yes*/
	private boolean failure;
	
	/** keeps track of the last added transition position*/
	private int head;
	
	/** keeps track of how many memory positions are occupied*/
	private int size;
	
	/** keeps track of the current stage in the creation of a new transition*/
	private int stage;
	
	
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
		assertStage(0);
		if (state_prev != null)
			add(new Transition(state_prev, state, action, reward, done || failure));
		state_prev = state;
	}
	
	/**
	 * Sets the most recent chosen action.
	 * 
	 * @param the action
	 */
	public void setAction(int action) {
		assertStage(1);
		this.action = action;
	}
	
	/**
	 * Sets the most recent received reward. If it is a terminal state, adds transition to memory.
	 * 
	 * @param the reward
	 */
	public void setReward(double reward, boolean done, boolean failure) {
		assertStage(2);
		this.reward = reward;
		this.done = done;
		this.failure = failure;
		
		if(done || failure) {
			add(new Transition (state_prev, null, action, reward, done || failure));
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
		failure = false;
		head = -1;
		size = 0;
		stage = 0;
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
	 * Checks that the setters for the state, action and reward are called in the correct order.
	 */
	public void assertStage(int s) {
		
		if (s != stage) {
			String info;
			switch (stage) {
			case 0:
				info = "State";
				break;
			case 1:
				info = "Action";
				break;
			case 2:
				info = "Reward and Done";
				break;
			default:
				info = null;
			}
			throw new IllegalStateException("Expected information:"  + info);
			
		} else {
			stage++;
			if (stage > 2)
				stage = 0;
		}
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
		float[][] nextStates = new float[batchSize][];
		int[] actions = new int[batchSize];
		double[] rewards = new double[batchSize];
		boolean [] dones = new boolean[batchSize];
		
		// Fills arrays with information from the circular buffer of transitions
		for (int i = 0; i< batchSize; i++) {

			states[i] = transitions[i].getState();
			// If next state is null (transition where episode ended), create array of zeros
			float[] nextState = transitions[i].getNextState();
			nextStates[i] = nextState != null ? nextState : new float[states[i].length];
			actions[i] = transitions[i].getAction();
			rewards[i] = transitions[i].getReward();
			dones[i] = transitions[i].isDone();
		}
		
		return new MemoryBatch(manager.create(states), manager.create(nextStates), manager.create(actions), manager.create(rewards), manager.create(dones));
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
