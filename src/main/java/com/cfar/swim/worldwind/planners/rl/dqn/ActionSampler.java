package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.Random;

import ai.djl.ndarray.NDArray;

/**
 * Contains different methods for action choosing during training and compilation of Deep Q-Learning planners
 * 
 * @author Rafaela Seguro
 *
 */

public class ActionSampler {
	
	/** Chooses action based on the epsilon greedy policy
	 * 
	 * @param the array of possible action indices
	 * @param a random number
	 * @param the value of epsilon
	 * 
	 * @return the index of the chosen action
	 * 
	 */
	public static int epsilonGreedy(NDArray qValues, Random rand, float epsilon) {
		float random = rand.nextFloat();
		if (random < epsilon) {
			//System.out.println("Epsilon is " + epsilon + " rand is " + random + " and action will be random");
			return rand.nextInt((int) qValues.size());
		} else {
			//System.out.println("Epsilon is " + epsilon + " rand is " + random + " and action will be greedy");
			return greedy(qValues);
		}
	}
	
	/** Chooses action based on the greedy policy
	 * 
	 * @param the array of possible action indices
	 * 
	 * @return the index of the chosen action
	 * 
	 */
	public static int greedy(NDArray qValues) {
		return (int) qValues.argMax().getLong();
	}

}
