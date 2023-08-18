package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.Random;

import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.types.Shape;

/**
 * Class that contains some extra methods to be used by the DQN planner.
 * 
 * @author Rafaela Seguro
 *
 */

public final class Helper {
	
	/** 
	 * 
	 * @param 
	 * @param 
	 * 
	 * @return 
	 * 
	 */
	public static NDArray gather(NDArray arr, int[] indices){
		boolean[][] mask = new boolean[(int) arr.size(0)][(int) arr.size(1)];
		for (int i = 0; i< indices.length; i++) {
			mask[i][indices[i]] = true;
		}
		NDArray booleanMask = arr.getManager().create(mask);
		for (int i = (int) booleanMask.getShape().dimension(); i < arr.getShape().dimension(); i++) {
			booleanMask = booleanMask.expandDims(i);
		}
		
		return arr.get(tile(booleanMask, arr.getShape())).reshape(Shape.update(arr.getShape(), 1, 1)).squeeze();
	}
	
	/** 
	 * 
	 * @param 
	 * @param
	 * 
	 * @return 
	 * 
	 */
	public static NDArray tile(NDArray arr, Shape shape) {
		for (int i = (int) arr.getShape().dimension(); i < shape.dimension(); i++) {
			arr = arr.expandDims(i);
		}
		return arr.broadcast(shape);
	}

}
