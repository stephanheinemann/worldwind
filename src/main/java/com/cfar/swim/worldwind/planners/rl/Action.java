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
package com.cfar.swim.worldwind.planners.rl;

import java.util.*;

/**
 * Enumerates the possible actions for an RL planner.
 * 
 * @author Rafaela Seguro
 *
 */
public enum Action {

	/**
	 * Direct flight towards target
	 */
	DIRECT(0, 0),
	
	/**
	 * Turns right at a 45 degree angle
	 */
	RIGHT45(0, 45),

	/**
	 * Turns right at a 60 degree angle
	 */
	RIGHT60(0, 60),
	
	/**
	 * Turns left at a 45 degree angle
	 */
	LEFT45(0, -45),
	
	/**
	 * Turns left at a 60 degree angle
	 */
	LEFT60(0, -60),
	
	/**
	 * Climbs at a 45 degree angle
	 */
	CLIMB45(1, 45),
	
	/**
	 * Descends at a 45 degree angle
	 */
	DESCEND45(2, 45);
	
	
	/** The action type (0 - turn, 1 - climb, 2 - descend)*/
	public int type;
	
	/** The angle associated with each action */
	public int angle;
	
	
	/** Construtcs an action element with the associated angle.
	 * 
	 * @param the associated angle
	 */
	private  Action (int type, int angle) {
		this.type = type;
		this.angle = angle;
	}
	
	
	/** Gets a random action from the available options.
	 * 
	 * @return a random action
	 */
	public static Action getRandom() {
		Random random = new Random();
		return values()[random.nextInt(values().length)];
	}
	
	/** Gets the action's type (0 - turn, 1 - climb, 2 - descend)
	 * 
	 * @return the action's type
	 */
	public int getType() {
		return this.type;
	}

}
