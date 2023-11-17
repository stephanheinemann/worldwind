/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.cfar.swim.worldwind.environments;

import java.time.ZonedDateTime;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planners.rl.RLObstacle;
import com.cfar.swim.worldwind.planners.rl.Snapshot;
import com.cfar.swim.worldwind.planners.rl.State;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a RL environment that can be sampled by sampling based motion
 * planning algorithms.
 * 
 * @author Rafaela Seguro
 */
public class RLEnvironment extends PlanningContinuum {
	
	/** in how many steps to divide from start to goal */
	private static final int STEP_DIVISION = 10;
	
	/** random number */
	private final Random rand = new Random();
	
	/** dimension of a state ID (input of the neural network) */
	private int dimOfState = State.ID_SIZE;
	
	/** the number of available actions in each state */
	private final int numOfActions = 11;
	
	/** the environment's start state */
	private State start = null;
	
	/** the environment's goal position */
	private Position goalPosition = null;
	
	/** the current state */
	private State state = null;
	
	/** the next state */
	private State nextState = null;
	
//	/** the index of the chosen action */
//	private int action = 0;
	
	/** the received reward */
	private double reward = 0.0;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean done = false;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean failure = false;
	
	/** stores the step size of the planner */
	private double stepSize = 0.0;
	
	/** set of random obstacles for training */
	private HashSet<Obstacle> trainingObstacles = new HashSet<>();
	
	/** the risk policy used for training */
	private RiskPolicy riskPolicy = RiskPolicy.AVOIDANCE;
	
	/** the cost policy used for training */
	private CostPolicy costPolicy = CostPolicy.AVERAGE;
	
	/** the current step in planning */
	int step = 0;

	
	/**
	 * Constructs a new RL environment based on a geometric box.
	 * 
	 * @param box the geometric box delimiting RL environment
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public RLEnvironment(Box box) {
		super(box);
	}
	
	/**
	 * Gets the start state.
	 * 
	 * @return the start state
	 */
	public State getStart() {
		return this.start;
	}
	
	/**
	 * Sets the start state.
	 * 
	 * @param the start state
	 */
	public void setStart(State start) {
		this.start = start;
	}
	
	/**
	 * Gets the goal position.
	 * 
	 * @return the goal position
	 */
	public Position getGoalPosition() {
		return this.goalPosition;
	}
	
	/**
	 * Sets the risk policy.
	 * 
	 * @param the risk policy
	 */
	public void setRiskPolicy(RiskPolicy riskPolicy) {
		this.riskPolicy = riskPolicy;
	}
	
	/**
	 * Sets the cost policy.
	 * 
	 * @param the cost policy
	 */
	public void setCostPolicy(CostPolicy costPolicy) {
		this.costPolicy = costPolicy;
	}
	
	/**
	 * Gets the size of a state's ID, which is the size of the network input.
	 * 
	 * @return the size of a state's ID
	 */
	public int getDimOfState() {
		return this.dimOfState;
	}
	
	/**
	 * Gets the number of available actions.
	 * 
	 * @return the number of available actions
	 */
	public int getNumOfActions() {
		return this.numOfActions;
	}
	
	/**
	 * Checks if the planner has reached the goal
	 * 
	 * @return true if it reached the goal, false otherwise
	 */
	public boolean isDone() {
		return this.done;
	}
	
	/**
	 * Checks if the planner has failed
	 * 
	 * @return true if it failed, false otherwise
	 */
	public boolean failed() {
		return this.failure;
	}

	
	/** Calculates the step size 
	 * 
	 * @param the start position
	 * @param the goal position
	 * */
	public double calculateStepSize(Position start, Position goal) {
		
		Vec4 goalPoint = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(goal));
		Vec4 startPoint = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(start));
		
		double distance = goalPoint.subtract3(startPoint).getLength3();
		this.stepSize = distance / STEP_DIVISION;
		
		return this.stepSize;
	}
	
	/**
	 * Gets the step size.
	 * 
	 * @return the step size
	 */
	public double getStepSize() {
		return this.stepSize;
	}
	
	/**
	 * Gets the cost policy.
	 * 
	 * @return the cost policy
	 */
	public CostPolicy getCostPolicy() {
		return this.costPolicy;
	}
	
	/** 
	 * Resets the environment with a new random start and goal
	 */
	public void resetRandom(boolean addObstacles) {
		
		// Starts by sampling random positions for the start and goal, following a uniform distribution
		Position startPosition = this.sampleRandomUniformPosition();
		this.goalPosition = startPosition;
		// Makes sure the start and goal are different
		while (this.goalPosition == startPosition)
			this.goalPosition = this.sampleRandomUniformPosition();
		
		// Sets the step size
		this.calculateStepSize(startPosition, this.goalPosition);
		
		// Adds obstacles to the environment
		if(addObstacles)
			this.createRandomObstacles(startPosition, this.goalPosition);
		
		// Creates the start state
		TreeSet<RLObstacle> obstacles = this.getCloseObstacles(startPosition, this.getTime());
		this.start = new State(startPosition, this.goalPosition, this, obstacles, this.getTime(), null);
		
		// Sets the state as the start
		this.state = this.start;
		
		// Sets the "done" and "failure" booleans to false
		this.done = false;
		this.failure = false;
		
	}
	
	/** 
	 * Initializes the environment to compute a path from a given origin to destination
	 * 
	 * @param the origin position
	 * @param the destination position
	 * @param the planner's risk policy
	 * @param the planner's cost policy
	 * @param the etd
	 */
	public void initializeEnvironment(Position origin, Position destination, RiskPolicy riskPolicy, CostPolicy costPolicy, ZonedDateTime etd) {
		
		this.unembedTrainingObstacles();
		
		// Sets the risk and cost policies
		this.riskPolicy = riskPolicy;
		this.costPolicy = costPolicy;
		
		this.goalPosition = destination;
		
		// Sets the step size
		this.calculateStepSize(origin, this.goalPosition);
		
		// Creates the start state
		TreeSet<RLObstacle> obstacles = this.getCloseObstacles(origin, etd);
		this.start = new State(origin, this.goalPosition, this, obstacles, etd, null);

		
		// Sets the state as the start
		this.state = this.start;
		
		// Sets the "done" and "failure" booleans to false
		this.done = false;
		this.failure = false;
		
	}
	
	
	/** 
	 * Determines the next state based on the chosen action
	 * 
	 * @param the action
	 * @param the aircraft for calculating ETOs based on capabilities
	 * 
	 * @return the snapshot that represents the environment state after the step
	 */
	public Snapshot step(int action, Aircraft aircraft) {
		
		// Determines the movement from the current state to the next based on action
		Vec4 movVector = this.getNewMoveVector(this.state.getMovVector(), this.state.getNormalizedRelativeGoal(), action);
		
		// Computes the next state's position
		Position nextStatePosition = this.getNewPosition(this.state.getPosition(), movVector.multiply3(this.stepSize));
		
		// If the next state is outside the environment, stays in place, and fails
		if (!this.contains(nextStatePosition)) {
			nextStatePosition = this.state.getPosition();
			//this.failure = true;
		}
		
		// Computes the next state's ETO
		Path leg = new Path(this.state.getPosition(), nextStatePosition);
		ZonedDateTime eto = aircraft.getCapabilities().getEstimatedTime(leg, this.getGlobe(), this.state.getEto());
		
		// Gets the set of surrounding obstacles of the next state
		TreeSet<RLObstacle> obstacles = this.getCloseObstacles(nextStatePosition, eto);
		
		// Creates the next state and adds it to the map
		this.nextState = new State(nextStatePosition, this.goalPosition, this, obstacles, eto, movVector);
		
		// Checks if it has reached goal
		if (this.toDistance(nextState.getNormalizedDistanceToGoal()) <= this.stepSize)
			this.done = true;
		
		// Calculates reward
		this.calculateReward();
		
		this.state = this.nextState;
		
		return new Snapshot(this.nextState, this.reward, this.done, this.failure);
	}
	

	/**
	 * Calculates the reward based on the next state
	 */
	protected void calculateReward() {
		
		double reward = 0;
		
		// If the goal has been reached
		if (this.isDone()) {
			this.reward = 150;
			return;
		}
		
		// If it tried to leave the environment
		if (this.nextState.getPosition() == this.state.getPosition()){
			this.reward = -30;
			return;
		}
		
		// If the cost is exceeded
		double cost = this.getLegCost(this.state.getPosition(), this.nextState.getPosition(), this.state.getEto(), 
				this.nextState.getEto(), this.costPolicy, this.riskPolicy);
		if (Double.POSITIVE_INFINITY == cost) {
			this.failure = true;
			this.reward = -150;
			return;
		}
		
		// If it gets too close to obstacle
		Iterator<RLObstacle> itr = this.nextState.getObstacles().iterator();
		RLObstacle current = null;
		while(itr.hasNext()) {
			current = itr.next();
			if (current.getDistanceToState() <= 2*this.nextState.getStepSize()) {
				reward += -10;
				break;
			}
		}
		
		// If it gets too close to environment boundary
		for(int i = 0; i < 6; i++) {
			if (this.nextState.getDistanceToEnv()[i] <= this.nextState.getStepSize()) {
				reward += -10;
				break;
			}
		}
		
		// Otherwise: step penalty +  distance to goal penalty/reward 
		reward += -10;
//		if (this.state.getNormalizedDistanceToGoal() <= this.nextState.getNormalizedDistanceToGoal()) {
//			reward += -10;
//		} else {
//			reward += 10;
//		}
		reward += 1 * (this.toDistance((this.state.getNormalizedDistanceToGoal() - this.nextState.getNormalizedDistanceToGoal())));
		
		this.reward = reward;
	
	}
	


	/** 
	 * Unembeds the training obstacles from the environment
	 */
	public void unembedTrainingObstacles() {
		
		for (Obstacle o : this.trainingObstacles) {
			this.unembed(o);
		}
		
		this.trainingObstacles.clear();
	}
	
	/** 
	 * Creates random obstacles and adds them to the planning continuum. Also sets a random cost and risk policy
	 */
	protected void createRandomObstacles(Position start, Position goal) {
		
		// Starts by unembeding training obstacles from previous episode
		this.unembedTrainingObstacles();
		
		// TODO: maybe add dynamic obstacles but for now they are static
		ZonedDateTime t1 = this.getTime().minusYears(1);
		ZonedDateTime t2 = this.getTime().plusYears(1);
		
		int numberOfObstacles = rand.nextInt(State.CONSIDERED_OBSTACLES);
		
		// Creates random obstacles and adds them to set
		for (int i=0; i<numberOfObstacles; i++) {
			
			Position center = this.sampleRandomUniformPosition();
			double radius = rand.nextDouble() * (this.getRadius() / 5);
			// TODO: add other types of obstacles, for now it is only sphere since it is the simplest to create
			ObstacleSphere obstacle = new ObstacleSphere(center, radius);
			
			double cost = rand.nextDouble() * 100;
			obstacle.setCostInterval(new CostInterval("ci", t1, t2, cost));
			
			this.trainingObstacles.add(obstacle);
		}
		
		 // 1/2 of the times, there is an obstacle right between start and goal, to make sure it learns to go around
		if (rand.nextInt(1)==0) {
			
			Vec4 startPoint = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(start));
			Vec4 goalPoint = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(goal));
			Vec4 halfStartToGoal = goalPoint.subtract3(startPoint).divide3(2d);
			Position center = this.getGlobe().computePositionFromPoint(this.transformBoxOriginToModel(startPoint.add3(halfStartToGoal)));

			double radius = rand.nextDouble() * (this.getRadius() / 5);
			ObstacleSphere obstacle = new ObstacleSphere(center, radius);
			
			double cost = rand.nextDouble() * 100;
			obstacle.setCostInterval(new CostInterval("ci", t1, t2, cost));
			
			this.trainingObstacles.add(obstacle);
			
		}
		
		// Embed training obstacles into environment
		for (Obstacle o : this.trainingObstacles) {
			this.embed(o);
		}
		
		// Sets random cost policy
		int costPolicy = rand.nextInt(2);
		this.costPolicy = CostPolicy.values()[costPolicy];
		
		// Sets random risk policy
		int riskPolicy = rand.nextInt(4);
		this.riskPolicy = RiskPolicy.values()[riskPolicy];

	}
	
	
	/** 
	 * Calculates the new movVector depending on the chosen action
	 * 
	 * @param the original move vector
	 * @param the action
	 * 
	 * @return the new movVector (normalized)
	 */
	public Vec4 getNewMoveVector (Vec4 originalVector, Vec4 relativeGoal, int action) {
		
		// TODO: create movements considering aircraft capabilities and not for 45 and 60 fixed angles

		double x = originalVector.x;
		double y = originalVector.y;
		double z = originalVector.z;
		double newX = x;
		double newY = y;
		double newZ = z;
		
		switch(action+1) {
//			// Go in direction of goal
//			case 0: 
//				newX = relativeGoal.x;
//				newY = relativeGoal.y;
//				newZ = relativeGoal.z;
//				break;
			// Go in same direction as previous
			case 1: 
				break;
			// Turn left 45 degrees
			case 2: 
				newZ = z * Math.cos(Math.toRadians(45)) + y * Math.sin(Math.toRadians(45));
				newY = -z * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
				break;
			// Turn left 60 degrees
			case 3: 
				newZ = z * Math.cos(Math.toRadians(60)) + y * Math.sin(Math.toRadians(60));
				newY = -z * Math.sin(Math.toRadians(60)) + y * Math.cos(Math.toRadians(60));
				break;
			// Turn right 45 degrees
			case 4: 
				newZ = z * Math.cos(Math.toRadians(45)) - y * Math.sin(Math.toRadians(45));
				newY = z * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
				break;
			// Turn right 60 degrees
			case 5: 
				newZ = z * Math.cos(Math.toRadians(60)) - y * Math.sin(Math.toRadians(60));
				newY = z * Math.sin(Math.toRadians(60)) + y * Math.cos(Math.toRadians(60));
				break;
			// Climb 45 degrees
			case 6: 
				newX = x + Math.tan(Math.toRadians(45));
				break;
			// Descend 45 degrees
			case 7:
				newX = x - Math.tan(Math.toRadians(45));
				break;
			// Up/Left 45 degrees
			case 8: 
				newX = x + Math.tan(Math.toRadians(45));
				newZ = z * Math.cos(Math.toRadians(45)) + y * Math.sin(Math.toRadians(45));
				newY = -z * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
				break;	
			// Up/Right 45 degrees
			case 9: 
				newX = x + Math.tan(Math.toRadians(45));
				newZ = z * Math.cos(Math.toRadians(45)) - y * Math.sin(Math.toRadians(45));
				newY = z * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
				break;	
			// Down/Left 45 degrees
			case 10:
				newX = x - Math.tan(Math.toRadians(45));
				newZ = z * Math.cos(Math.toRadians(45)) + y * Math.sin(Math.toRadians(45));
				newY = -z * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
				break;	
			// Down/Right 45 degrees
			case 11:
				newX = x - Math.tan(Math.toRadians(45));
				newZ = z * Math.cos(Math.toRadians(45)) - y * Math.sin(Math.toRadians(45));
				newY = z * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
				break;
			// Default
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
	protected TreeSet<RLObstacle> getCloseObstacles(Position position, ZonedDateTime eto) {
		
		TreeSet<RLObstacle> interferingObstacles = new TreeSet<RLObstacle>(Comparator.comparingDouble(RLObstacle::getDistanceToState));		
		
		// Goes through all the obstacles in the environment + the random ones
		for (Obstacle obstacle : this.getObstacles()) {
		
			// Checks if it the obstacle's cost interval contains the state's eto
			if(obstacle.getCostInterval().contains(eto)) {
				
				// Checks if the cost obstacle satisfies the risk policy and adds to set if it doesn't
				if (!this.riskPolicy.satisfies(obstacle.getCostInterval().getCost())) {
					
					RLObstacle newObstacle = new RLObstacle(position, obstacle, this);
					interferingObstacles.add(newObstacle);
				}
			}
		}
		
		return interferingObstacles;
	}
	
	/** 
	 * Gets the vector that points from one position to another, which represents
	 * their relative position
	 * 
	 * @param position that acts as origin
	 * @param other position
	 * 
	 * @return the normalized vector
	 * 
	 */
	public Vec4 getRelativeVector(Position origin, Position other) {
		
		Vec4 originPoint = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(origin));
		Vec4 otherPoint = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(other));
		
//		Vec4 originPoint = this.getGlobe().computePointFromPosition(origin);
//		Vec4 otherPoint = this.getGlobe().computePointFromPosition(other);
		
		Vec4 relativeVector = otherPoint.subtract3(originPoint);
		
		return relativeVector;
	}
	
	/** 
	 * Gets the normalized distance between two positions in box model
	 * 
	 * @param position one
	 * @param position two
	 * 
	 * @return the normalized distance
	 * 
	 */
	public double getNormalizedBoxDistance(Position p1, Position p2) {
		
		Vec4 point1 = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(p1));
		Vec4 point2 = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(p2));

//		Vec4 point1 = this.getGlobe().computePointFromPosition(p1);
//		Vec4 point2 = this.getGlobe().computePointFromPosition(p2);
		
		Vec4 relativeVector = point1.subtract3(point2);
		
		double distance = relativeVector.getLength3();
		
		return this.toNormalizedDistance(distance);
	}
	
	/** 
	 * Gets the normalized distances from a position in box model to the box axes
	 * 
	 * @param position 
	 * 
	 * @return the normalized distances
	 * 
	 */
	public float[] getDistancesToEnv(Position position) {
		
		float[] distancesToEnv = new float[6];
		
		Vec4 boxPoint = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(position));
		
		distancesToEnv[0] = (float) this.toNormalizedDistance(boxPoint.x);
		distancesToEnv[1] = (float) this.toNormalizedDistance(this.getRLength() - boxPoint.x);
		distancesToEnv[2] = (float) this.toNormalizedDistance(boxPoint.y);
		distancesToEnv[3] = (float) this.toNormalizedDistance(this.getSLength() - boxPoint.y);
		distancesToEnv[4] = (float) this.toNormalizedDistance(boxPoint.z);
		distancesToEnv[5] = (float) this.toNormalizedDistance(this.getTLength() - boxPoint.z);
		
//		Vec4[] corners = this.getCorners();
//		Vec4 min = corners[0];
//		Vec4 max = corners[6];
//		Vec4 point = this.getGlobe().computePointFromPosition(position);
//		
//		distancesToEnv[0] = (float) this.toNormalizedDistance(point.x - min.x);
//		distancesToEnv[1] = (float) this.toNormalizedDistance(max.x - point.x);
//		distancesToEnv[2] = (float) this.toNormalizedDistance(point.y - min.y);
//		distancesToEnv[3] = (float) this.toNormalizedDistance(max.y - point.y);
//		distancesToEnv[4] = (float) this.toNormalizedDistance(point.z - min.z);
//		distancesToEnv[5] = (float) this.toNormalizedDistance(max.z - point.z);
		
		return distancesToEnv;
	}
	
	/** 
	 * Gets the new position when applying a movement vector to another position
	 * 
	 * @param the original position 
	 * @param the movement vector
	 * 
	 * @return the new position
	 * 
	 */
	public Position getNewPosition(Position position, Vec4 move) {
		
		Vec4 point = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(position));
//		Vec4 goalPoint = this.transformModelToBoxOrigin(this.getGlobe().computePointFromPosition(goalPosition));

//		Vec4 point = this.getGlobe().computePointFromPosition(position);
//		Vec4 point2 = point.add3(move);
		
	//	Position newPosition = this.getGlobe().computePositionFromPoint(point.add3(move));
		Position newPosition = this.getGlobe().computePositionFromPoint(this.transformBoxOriginToModel(point.add3(move)));
		
		return newPosition;
	}
	
}
