package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.TreeSet;
import java.util.Random;
import java.util.Set;

import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
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
	
	/** 
	 * Calculates the new movVector depending on the chosen action
	 * 
	 * @param the original move vector
	 * @param the action
	 * 
	 * @return the new movVector (normalized)
	 */
	public static Vec4 getNewMoveVector (Vec4 originalVector, Vec4 relativeGoal, int action) {
		
		// TODO: create movements considering aircraft capabilities and not for 45 and 60 fixed angles

		double x = originalVector.x;
		double y = originalVector.y;
		double z = originalVector.z;
		double newX = x;
		double newY = y;
		double newZ = z;
		
		switch(action) {
			// Go in direction of goal 
			case 0: 
				newX = relativeGoal.x;
				newY = relativeGoal.y;
				newZ = relativeGoal.z;
				break;
			// Turn right 45 degrees
			case 1: 
				newX = x * Math.cos(Math.toRadians(45)) + y * Math.sin(Math.toRadians(45));
				newY = -x * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
//				newX = 1;
//				newY = 0;
//				newZ = 0;
				break;
			// Turn right 60 degrees
			case 2: 
				newX = x * Math.cos(Math.toRadians(60)) + y * Math.sin(Math.toRadians(60));
				newY = -x * Math.sin(Math.toRadians(60)) + y * Math.cos(Math.toRadians(60));
//				newX = -1;
//				newY = 0;
//				newZ = 0;
				break;
			// Turn left 45 degrees
			case 3: 
				newX = x * Math.cos(Math.toRadians(45)) - y * Math.sin(Math.toRadians(45));
				newY = x * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
//				newX = 0;
//				newY = 1;
//				newZ = 0;
				break;
			// Turn left 60 degrees
			case 4: 
				newX = x * Math.cos(Math.toRadians(60)) - y * Math.sin(Math.toRadians(60));
				newY = x * Math.sin(Math.toRadians(60)) + y * Math.cos(Math.toRadians(60));
//				newX = 0;
//				newY = -1;
//				newZ = 0;
				break;
			 // Climb 45 degrees
			case 5: 
				newZ = z + Math.tan(Math.toRadians(45));
//				newX = 0;
//				newY = 0;
//				newZ = 1;
				break;
			// Descend 45 degrees
			case 6:
				newZ = z - Math.tan(Math.toRadians(45));
//				newX = 0;
//				newY = 0;
//				newZ = -1;
				break;
			default:
		}
		
		Vec4 newVector = new Vec4(newX, newY, newZ);
		
		return newVector.normalize3();
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
	public static TreeSet<DQNObstacle> getCloseObstacles(PrecisionPosition position, PlanningContinuum environment,
			HashSet<Obstacle> trainingObstacles ,RiskPolicy riskPolicy) {
		
		TreeSet<DQNObstacle> interferingObstacles = new TreeSet<DQNObstacle>(Comparator.comparingDouble(DQNObstacle::getDistanceToState));
		
		trainingObstacles.addAll(environment.getObstacles());
		
		
		// Goes through all the obstacles in the environment + the random ones
		for (Obstacle obstacle : trainingObstacles) {
			
//			// Checks if it interferes with the obstacle representing the state
//			if (obstacle.intersects(environment.getGlobe(), stateObstacle)) {
				
				// Checks if the cost obstacle satisfies the risk policy and adds to set if it doesn't
				if (!riskPolicy.satisfies(obstacle.getCostInterval().getCost())) {
					
					DQNObstacle newObstacle = new DQNObstacle(position, obstacle, environment);
					interferingObstacles.add(newObstacle);
				}
//			}
		}
		
		return interferingObstacles;
	}

}


