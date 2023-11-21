package com.cfar.swim.worldwind.planners.rl.priordqn;

import java.util.Arrays;
import java.util.Random;

import com.cfar.swim.worldwind.planners.rl.MemoryBatch;
import com.cfar.swim.worldwind.planners.rl.Transition;

import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDManager;

/**
 * Realizes a replay memory object to store transitions and to be used by the Deep Neural Network
 * 
 * @author Rafaela Seguro
 *
 */

public class PrioritizedMemory {
	
	/** random number */
	private final Random rand = new Random(0);
	
	/** the memory's capacity */
	private int capacity = 0;
	
	/** the array to store transitions*/
	private Transition[] memory;
	
	/** stores the data associated to the stored transitions*/
	private TransitionData[] memoryData;
	
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
	
	/** keeps track of the total amount of transitions that have been stored*/
	private int totalTransitions;
	
	/** keeps track of how many memory positions are occupied*/
	private int size;
	
	/** keeps track of the current stage in the creation of a new transition*/
	private int stage;
	
	/** parameter that determines how much prioritization is used*/
	private double alpha;
	
	/** the sum of the priorities of the stored transitions (with exponent alpha)*/
	private double prioritySum = 0;
	
	/** the max of the priorities of the stored transitions*/
	private double priorityMax = 1.0;
	
	/** the max of the weights of the stored transitions*/
	private double weightMax = 0.0;
	
//	/** max initial priority for newly added transitions*/
//	private final double maxPriority = 1.0;
	
	
	/** Constructs a replay memory object
	 * 
	 * @param the memory's capacity
	 * @param the alpha parameter
	 */
	public PrioritizedMemory(int capacity, double alpha) {
		this.capacity = capacity;
		this.alpha = alpha;
		this.memory = new Transition[capacity];
		this.memoryData = new TransitionData[capacity];
		
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
	 * Sets the most recent received reward.
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
		totalTransitions = -1;
		size = 0;
		stage = 0;
	}
	
	/**
	 * Adds a transition to the replay memory.
	 * 
	 * @param the transition
	 */
	public void add(Transition transition) {
		
		@SuppressWarnings("unused")
		TransitionData temp = null;
		
		totalTransitions += 1;
		head = totalTransitions % capacity;
		
		// If memory is full the new transition replaces an old one and the new max might need to be found
		if (totalTransitions > capacity) {
			
			temp = memoryData[head];
			prioritySum -= Math.pow(temp.getPriority(), alpha);
			
			if (temp.getPriority() == priorityMax) {
				memoryData[head].setPriority(0);
				priorityMax = Arrays.stream(memoryData).mapToDouble(TransitionData::getPriority).max().orElseThrow();
			}
			if (temp.getWeight() == weightMax) {
				memoryData[head].setWeight(0);
				weightMax = Arrays.stream(memoryData).mapToDouble(TransitionData::getWeight).max().orElseThrow();
			}
		}
		
		double priority = priorityMax;
		double weight = weightMax;
		prioritySum += Math.pow(priority, alpha);
		double probability = Math.pow(priority, alpha) / prioritySum;
		
		transition.setIndex(head);
		memory[head] = transition;
		memoryData[head] = new TransitionData(head, priority, probability, weight);
		
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
	 * Samples a chunk of transitions from the memory with the size indicated, using
	 * proportional prioritization
	 * 
	 * @param the size of the sample
	 * @param the priority scale (value that controls the impact of priorities on the sampling distribution)
	 * 
	 * @return the sample and the transition's indices in memory
	 */
	public Transition[] sample(int sampleSize) {
		
		Transition[] sample = new Transition[sampleSize];
		double totalProbability = 0.0;
		double probability = 0.0;
		for (int j = 0; j < size; j++) {
			probability = Math.pow(memoryData[j].getPriority(), alpha) / prioritySum;
			memoryData[j].setProbability(probability);
			totalProbability += probability;
		}
		
		for (int i = 0; i<sampleSize; i++) {
			double samplePoint = rand.nextDouble() * totalProbability;
			int transitionIndex = -1;
			double currentSum = 0;
			
			for (int j = 0; j < size; j++) {
				currentSum += memoryData[j].getProbability();
				if (currentSum >= samplePoint) {
					transitionIndex = j;
					break;
				}
			}
			sample[i] = memory[transitionIndex];
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
		double[] indices = new double[batchSize];
		
		// Fills arrays with information from the circular buffer of transitions
		for (int i = 0; i< batchSize; i++) {

			states[i] = transitions[i].getState();
			// If next state is null (transition where episode ended), create array of zeros
			float[] nextState = transitions[i].getNextState();
			nextStates[i] = nextState != null ? nextState : new float[states[i].length];
			actions[i] = transitions[i].getAction();
			rewards[i] = transitions[i].getReward();
			dones[i] = transitions[i].isDone();
			indices[i] = transitions[i].getIndex();
		}
		
		return new MemoryBatch(manager.create(states), manager.create(nextStates), manager.create(actions), manager.create(rewards), 
				manager.create(dones), manager.create(indices));
	}
	
	
	/**
	 * Samples a random memory batch
	 * 
	 * @param the sample size
	 * @param the priority scale (value that controls the impact of priorities on the sampling distribution)
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
	
	

	
	/**
	 * Updates the data of the sampled transitions
	 * 
	 * @param the indices of the transitions
	 * @param the new priority values
	 * @param beta
	 * 
	 * @return the new values for the weights
	 */
	public double[] updateTransitionData(double[] indices, double[] newPriorities, double beta) {
		
		double[] newWeights = new double[indices.length];
		TransitionData temp;
		int j = 0;
		
		for (double index : indices) {
			
			int i = (int) index;
			temp = memoryData[i];
			
			prioritySum -= Math.pow(memoryData[i].getPriority(), alpha);
			memoryData[i].setPriority(newPriorities[j]);
			prioritySum += Math.pow(memoryData[i].getPriority(), alpha);
			double newProbability = Math.pow(newPriorities[j], alpha) / prioritySum;
			memoryData[i].setProbability(newProbability);
			newWeights[j] = Math.pow((size * newProbability), -beta);
			memoryData[i].setWeight(newWeights[j]);
			
			// Updates max values in case they were changed
			if (temp.getPriority() == priorityMax) {
				priorityMax = Arrays.stream(memoryData).mapToDouble(TransitionData::getPriority).max().orElseThrow();
			}
			if (temp.getWeight() == weightMax) {
				weightMax = Arrays.stream(memoryData).mapToDouble(TransitionData::getWeight).max().orElseThrow();
			}
			
			j++;
			
		}
		
		return newWeights;
	}
	

}
