package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.Random;
import java.util.Set;

import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planning.RiskPolicy;
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
	 * Method to broadcast and NDArray to match a specified shape
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
	
	/** 
	 * Creates the list of possible actions the agent can take
	 * 
	 * @return the list of possible actions
	 * 
	 */
	public static ArrayList<Vec4>  listOfActions() {
		
		ArrayList<Vec4> listOfActions = new ArrayList<Vec4> ();
		
		// The action with index 0 corresponds to going in the direction of the goal
		listOfActions.add(new Vec4(0,0,0));
		
		// The rest of the list is populated with all possible actions with angles 0, 22.5, 45, 67.5 and 90
//		double[] thetaValues = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180};
//		double[] alphaValues = {0, 22.5, 45, 67.5, 90};
		double[] thetaValues = {0, 90, 180};
		double[] alphaValues = {0, 90};
		double x, y, z;
		
		for (double theta : thetaValues){
			for (double alpha : alphaValues) {
				x = Math.sin(alpha) * Math.cos(theta);
				y = Math.sin(alpha) * Math.sin(theta);
				z = Math.cos(alpha);
				
				listOfActions.add(new Vec4(x, y, z));
			}
		}
		
		return listOfActions;
	}
	
	/** 
	 * Creates a set of the obstacles that are in close proximity to the current state
	 * and don't satisfy the current risk policy. 
	 * 
	 * @param the state's position
	 * @param the aircraft's radius
	 * @param the environment
	 * @param the risk policy
	 * 
	 * @return the set of DQN obstacles ordered by their distance to the state
	 * 
	 */
	public static Set<DQNObstacle> getCloseObstacles(Position position, double aircraftRadius, Environment environment,
			RiskPolicy riskPolicy) {
		
		Set<DQNObstacle> interferingObstacles = new TreeSet<DQNObstacle>(Comparator.comparingDouble(DQNObstacle::getDistanceToState));
		
		// Creates a new sphere obstacle with center in the state's position and triple the aircraft's radius
		ObstacleSphere stateObstacle = new ObstacleSphere(position, 3*aircraftRadius);
		
		// Goes through all the obstacles in the environment
		for (Obstacle obstacle : environment.getObstacles()) {
			
			// Checks if it interferes with the obstacle representing the state
			if (obstacle.intersects(environment.getGlobe(), stateObstacle)) {
				
				// Checks if the cost obstacle satisfies the risk policy
				if (!riskPolicy.satisfies(obstacle.getCostInterval().getCost())) {
					
					DQNObstacle newObstacle = new DQNObstacle(position, obstacle, environment.getGlobe());
					interferingObstacles.add(newObstacle);
				}
			}
		}
		
		return interferingObstacles;
	}
	
	/** 
	 * Flattens an array of arrays of floats 
	 * 
	 * @param the array of arrays
	 * 
	 * @return the flattened array
	 * 
	 */
	public static float[] flattenArray(float[][] nestedArray) {
	    int totalLength = 0;

	    // Calculate the total length of the flat array
	    for (float[] arr : nestedArray) {
	        totalLength += arr.length;
	    }

	    // Create the flat array with the calculated length
	    float[] flatArray = new float[totalLength];

	    int flatIndex = 0;

	    // Iterate through the nested arrays and copy elements to the flat array
	    for (float[] arr : nestedArray) {
	        for (float value : arr) {
	            flatArray[flatIndex] = value;
	            flatIndex++;
	        }
	    }

	    return flatArray;
	}

}


