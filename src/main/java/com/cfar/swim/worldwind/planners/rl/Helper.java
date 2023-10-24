package com.cfar.swim.worldwind.planners.rl;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.TreeSet;
import java.util.Random;
import java.util.Set;

import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;

import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.types.Shape;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;

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
	
//	/** 
//	 * Creates the list of possible actions the agent can take
//	 * 
//	 * @return the list of possible actions
//	 * 
//	 */
//	public static ArrayList<Vec4>  listOfActions() {
//		
//		ArrayList<Vec4> listOfActions = new ArrayList<Vec4> ();
//		
//		// Keep direction
//		listOfActions.add(new Vec4(0,0,0));
//		// Climb 45 degrees
//		listOfActions.add(new Vec4(0,0,1));
//		// Descend 45 degrees
//		listOfActions.add(new Vec4(0,0,-1));
//		// Turn left 45 degrees
//		listOfActions.add(new Vec4(0,1,0));
//		// Turn left 60 degrees
//		listOfActions.add(new Vec4(0,-1,0));
//		listOfActions.add(new Vec4(0,0,1));
//		listOfActions.add(new Vec4(0,0,-1));
//		
//		// The rest of the list is populated with all possible actions with angles 0, 22.5, 45, 67.5 and 90
////		double[] thetaValues = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180};
////		double[] alphaValues = {0, 22.5, 45, 67.5, 90};
////		double[] thetaValues = {0, 90, 180};
////		double[] alphaValues = {0, 90};
////		double x, y, z;
////		
////		for (double theta : thetaValues){
////			for (double alpha : alphaValues) {
////				x = Math.sin(Math.toRadians(alpha)) * Math.cos(Math.toRadians(theta));
////				y = Math.sin(Math.toRadians(alpha)) * Math.sin(Math.toRadians(theta));
////				z = Math.cos(Math.toRadians(alpha));
////				
////				listOfActions.add(new Vec4(x, y, z));
////			}
////		}
//		
//		return listOfActions;
//	}


}


