package com.cfar.swim.worldwind.planners.rl;

import java.util.Arrays;
import java.util.Collections;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Realizes an entry line of the Q-table, corresponding to one state. Saves the Q-values for each
 * available action in one state
 * 
 * @author Rafaela Seguro
 *
 */

public class QLine {
	
	/** the state this line corresponds to */     
	private RLWaypoint state = null;
	
	/** the number of actions available in this state */
	private int actions = 0;
	
	/** array of Q-values */
	private double[] qValues;
	
	/** random number */
	Random rand = new Random();
	
	
	/** Constructs a Q-table entry, with an array of Q-values that has as 
	 * entry for each available action in the current state
	 * 
	 * @param the corresponding state
	 * @param the number of available actions
	 */
	public QLine(RLWaypoint state, int actions) {
		this.setState(state);
		this.actions = actions;
		this.qValues = new double[this.actions];
	}
	
	/**
	 * Gets the state this Q-line corresponds to.
	 * 
	 * @return state the Q-line corresponds to
	 */
	public RLWaypoint getState() {
		return state;
	}

	/**
	 * Sets the state this Q-line corresponds to.
	 * 
	 * @param state the Q-line corresponds to
	 */
	public void setState(RLWaypoint state) {
		this.state = state;
	}
	
	/**
	 * Gets the number of actions for this Q-line's state.
	 * 
	 * @return the number of actions
	 */
	public int getActions() {
		return actions;
	}
	
	/** Initializes all Q-values as zero 
	 */
	public void initQValues() {
		for (int i = 0; i < this.qValues.length; i++) {
			this.qValues[i] = 0d;
		}
	}
	
	/** Updates Q-value for a specific action
	 * 
	 * @param the action
	 * @param the Q-value
	 */
	public void updateQValue(int action, double qValue) {
		this.qValues[action] = qValue;
	}
	
	/** Gets Q-value for a specific action
	 * 
	 * @param the action
	 * 
	 * @return the Q-value
	 */
	public double getQValue(int action) {
		return this.qValues[action];
	}
	
	/** Gets action index correspondent to max Q-value. Random in case of tie.
	 * 
	 * @return action index correspondent to the max Q-value
	 */
	public int getMaxQValue() {
		double maxQValue = -500000;
		int index = 0;
		for (int i = 0; i < actions; i++) {
			if (qValues[i] > maxQValue) {
				maxQValue = qValues[i];
				index = i;
			}
			else if (qValues[i] == maxQValue) {
				 double r = rand.nextDouble();
				 if (r>0.5) index = i;
			}
		}
		return index;
	}
	
	/** Gets action index correspondent to the i'th max Q-value. For example, i=1
	 * returns max qValue, while i=2 returns second max qValue, and so on
	 * 
	 * @return action index correspondent to the i'th max Q-value
	 */
	public int getMaxQValue(int i) {
		int orderedActions[] = new int[actions];
		for (int j=0; j<actions; orderedActions[j] = j++);
		for (int m=0; m<orderedActions.length-1; m++) {
			for (int n=0; n<orderedActions.length-m-1; n++) {
				if(qValues[orderedActions[n]] > qValues[orderedActions[n+1]]) {
					int temp = orderedActions[n];
					orderedActions[n] = orderedActions[n+1];
					orderedActions[n+1] = temp;
				}
			}
		}
		return orderedActions[actions-i];
	}
	
	/** Gets action index correspondent to max Q-value. Random in case of tie.
	 * 
	 * @return action index correspondent to the max Q-value
	 */
	public int getEGreedyAction(double epsilon) {
		int index = 0;
		double r = rand.nextDouble();
		if (r<epsilon) {
			index = ThreadLocalRandom.current().nextInt(0, this.actions);
		} else {
			index = getMaxQValue();
		}
		return index;
	}

}
