package com.cfar.swim.worldwind.planners.rl;

import com.cfar.swim.worldwind.planning.Waypoint;

/**
 * Realizes an entry line of the Q-table, corresponding to one state. Saves the Q-values for each
 * available action in one state
 * 
 * @author Rafaela Seguro
 *
 */

public class QLine {
	
	/** the state this line corresponds to */     
	private Waypoint state = null;
	
	/** the number of actions available in this state */
	private int actions = 0;
	
	/** array of Q-values */
	private float[] qValues;
	
	
	/** Constructs a Q-table entry, with an array of Q-values that has as 
	 * entry for each available action in the current state
	 * 
	 * @param the corresponding state
	 * @param the number of available actions
	 */
	public QLine(Waypoint state, int actions) {
		this.state = state;
		this.actions = actions;
		this.qValues = new float[this.actions];
	}
	
	/** Initializes all Q-values as zero 
	 */
	protected void initQValues() {
		for (int i = 0; i < this.qValues.length; i++) {
			this.qValues[i] = 0;
		}
	}
	
	/** Updates Q-value for a specific action
	 * 
	 * @param the action
	 * @param the Q-value
	 */
	protected void updateQValue(int action, float qValue) {
		this.qValues[action] = qValue;
	}

}
