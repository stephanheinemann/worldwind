package com.cfar.swim.worldwind.planners.rl;

import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.types.Shape;

/**
 * Defines a class containing extra methods that are called by the DQN planner
 * 
 * @author Rafaela Seguro
 *
 */

public final class Helper {
	
	/** 
	 * Gathers elements from and NDArray based on specified indexes 
	 * 
	 * @param predicted QValues the NDArray containing the predicted Q values
	 * @param actions the indexes of the chosen actions
	 * 
	 * @return NDArray with the gathered elements
	 * 
	 */
	public static NDArray gather(NDArray predictedQValues, int[] actions){
		// Creates boolean mask with same shape as the predicted Q-values array (for each state a Q-value per action)
		boolean[][] mask = new boolean[(int) predictedQValues.size(0)][(int) predictedQValues.size(1)];
		// Sets to true the entries corresponding to the actions that were chosen in each case     
		for (int i = 0; i< actions.length; i++) {
			mask[i][actions[i]] = true;
		}
		// Creates NDArray with shape of mask and makes sure it has the right dimensions
		NDArray booleanMask = predictedQValues.getManager().create(mask);
		for (int i = (int) booleanMask.getShape().dimension(); i < predictedQValues.getShape().dimension(); i++) {
			booleanMask = booleanMask.expandDims(i);
		}
		// Returns an NDArray with the gathered elements corresponding to the chosen actions
		return predictedQValues.get(tile(booleanMask, predictedQValues.getShape())).reshape(Shape.update(predictedQValues.getShape(), 1, 1)).squeeze();
	}
	
	/** 
	 * Method to broadcast an NDArray to match a specified shape
	 * 
	 * @param arr the NDArray
	 * @param shape the specified shape
	 * 
	 * @return the NDArray with the specified shape
	 * 
	 */
	public static NDArray tile(NDArray arr, Shape shape) {
		for (int i = (int) arr.getShape().dimension(); i < shape.dimension(); i++) {
			arr = arr.expandDims(i);
		}
		return arr.broadcast(shape);
	}

}


