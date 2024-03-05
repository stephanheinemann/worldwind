package com.cfar.swim.worldwind.planners.rl.priorddqn;

/**
 * Realizes a transition data object to store the index in memory, the priority, the probability of being 
 * sampled and the weight of a transition (to be used for Prioritized Experience Replay)
 * 
 * @author Rafaela Seguro
 *
 */

public class TransitionData {
	
	/** stores the transition's index in memory */
	private int index = 0;
	
	/** the priority */     
	private double priority = 0.0;
	
	/** the probability of being sampled */     
	private double probability = 0.0;
	
	/** the bias correcting weight */     
	private double weight = 0.0;
	
	
	/** Constructs a transition tuple
	 * 
	 * @param the index
	 * @param the priority
	 * @param the probability
	 * @param the weight
	 */
	public TransitionData(int index, double priority, double probability, double weight) {
		
		this.index = index;
		this.priority = priority;
		this.probability = probability;
		this.weight = weight;
	}
	
	/**
	 * Gets the index of the corresponding transition.
	 * 
	 * @return the index
	 */
	public int getIndex() {
		return this.index;
	}

	/**
	 * Sets the transition's priority.
	 * 
	 * @param the priority
	 */
	public void setPriority(double priority) {
		this.priority = priority;
	}
	
	/**
	 * Gets the transition's priority.
	 * 
	 * @return the priority
	 */
	public double getPriority() {
		return this.priority;
	}
	
	/**
	 * Sets the transition's probability.
	 * 
	 * @param the probability
	 */
	public void setProbability(double probability) {
		this.probability = probability;
	}
	
	/**
	 * Gets the transition's probability.
	 * 
	 * @return the probability
	 */
	public double getProbability() {
		return this.probability;
	}
	
	/**
	 * Sets the transition's weight.
	 * 
	 * @param the weight
	 */
	public void setWeight(double weight) {
		this.weight = weight;
	}
	
	/**
	 * Gets the transition's weight.
	 * 
	 * @return the weight
	 */
	public double getWeight() {
		return this.weight;
	}

}
