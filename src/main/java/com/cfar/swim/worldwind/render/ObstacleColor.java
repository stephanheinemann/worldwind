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
package com.cfar.swim.worldwind.render;

import java.awt.Color;

/**
 * Realizes an obstacle color following common severity thresholds of
 * weather radar color coding standards.
 * 
 * @author Stephan Heinemann
 *
 */
public class ObstacleColor {
	
	private final static int THRESHOLD_NONE = 0;
	private final static int THRESHOLD_LIGHT_1 = 5;
	private final static int THRESHOLD_LIGHT_2 = 10;
	private final static int THRESHOLD_LIGHT_3 = 15;
	private final static int THRESHOLD_LIGHT_4 = 20;
	private final static int THRESHOLD_LIGHT_5 = 25;
	private final static int THRESHOLD_MODERATE_1 = 30;
	private final static int THRESHOLD_MODERATE_2 = 35;
	private final static int THRESHOLD_MODERATE_3 = 40;
	private final static int THRESHOLD_MODERATE_4 = 45;
	private final static int THRESHOLD_HEAVY_1 = 50;
	private final static int THRESHOLD_HEAVY_2 = 55;
	private final static int THRESHOLD_HEAVY_3 = 60;
	private final static int THRESHOLD_HEAVY_4 = 65;
	private final static int THRESHOLD_EXTREME_1 = 70;
	private final static int THRESHOLD_EXTREME_2 = 75;

	/**
	 * Gets the color for a specified cost.
	 * 
	 * @param cost the cost
	 * @return the color for the specified cost
	 */
	public static Color getColor(double cost) {
		if (cost >= ObstacleColor.THRESHOLD_EXTREME_2) {
			return Color.WHITE;
		}
		if (cost >= ObstacleColor.THRESHOLD_EXTREME_1) {
			return Color.MAGENTA.darker();
		}
		if (cost >= ObstacleColor.THRESHOLD_HEAVY_4) {
			return Color.MAGENTA;
		}
		if (cost >= ObstacleColor.THRESHOLD_HEAVY_3) {
			return Color.RED.darker();
		}
		if (cost >= ObstacleColor.THRESHOLD_HEAVY_2) {
			return Color.RED;
		}
		if (cost >= ObstacleColor.THRESHOLD_HEAVY_1) {
			return Color.RED.brighter();
		}
		if (cost >= ObstacleColor.THRESHOLD_MODERATE_4) {
			return Color.ORANGE;
		}
		if (cost >= ObstacleColor.THRESHOLD_MODERATE_3) {
			return Color.YELLOW.darker();
		}
		if (cost >= ObstacleColor.THRESHOLD_MODERATE_2) {
			return Color.YELLOW;
		}
		if (cost >= ObstacleColor.THRESHOLD_MODERATE_1) {
			return Color.GREEN.darker();
		}
		if (cost >= ObstacleColor.THRESHOLD_LIGHT_5) {
			return Color.GREEN;
		}
		if (cost >= ObstacleColor.THRESHOLD_LIGHT_4) {
			return Color.GREEN.brighter();
		}
		if (cost >= ObstacleColor.THRESHOLD_LIGHT_3) {
			return Color.BLUE.darker();
		}
		if (cost >= ObstacleColor.THRESHOLD_LIGHT_2) {
			return Color.CYAN.darker();
		}
		if (cost >= ObstacleColor.THRESHOLD_LIGHT_1) {
			return Color.CYAN;
		}
		if (cost >= ObstacleColor.THRESHOLD_NONE) {
			return Color.DARK_GRAY;
		}
		
		return Color.BLACK;
	}
	
}
