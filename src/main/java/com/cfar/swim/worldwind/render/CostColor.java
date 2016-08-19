package com.cfar.swim.worldwind.render;

import java.awt.Color;

public class CostColor {
	
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

	public static Color getColor(int cost) {
		if (cost >= CostColor.THRESHOLD_EXTREME_2) {
			return Color.WHITE;
		}
		if (cost >= CostColor.THRESHOLD_EXTREME_1) {
			return Color.MAGENTA.darker();
		}
		if (cost >= CostColor.THRESHOLD_HEAVY_4) {
			return Color.MAGENTA;
		}
		if (cost >= CostColor.THRESHOLD_HEAVY_3) {
			return Color.RED.darker();
		}
		if (cost >= CostColor.THRESHOLD_HEAVY_2) {
			return Color.RED;
		}
		if (cost >= CostColor.THRESHOLD_HEAVY_1) {
			return Color.RED.brighter();
		}
		if (cost >= CostColor.THRESHOLD_MODERATE_4) {
			return Color.ORANGE;
		}
		if (cost >= CostColor.THRESHOLD_MODERATE_3) {
			return Color.YELLOW.darker();
		}
		if (cost >= CostColor.THRESHOLD_MODERATE_2) {
			return Color.YELLOW;
		}
		if (cost >= CostColor.THRESHOLD_MODERATE_1) {
			return Color.GREEN.darker();
		}
		if (cost >= CostColor.THRESHOLD_LIGHT_5) {
			return Color.GREEN;
		}
		if (cost >= CostColor.THRESHOLD_LIGHT_4) {
			return Color.GREEN.brighter();
		}
		if (cost >= CostColor.THRESHOLD_LIGHT_3) {
			return Color.BLUE.darker();
		}
		if (cost >= CostColor.THRESHOLD_LIGHT_2) {
			return Color.CYAN.darker();
		}
		if (cost >= CostColor.THRESHOLD_LIGHT_1) {
			return Color.CYAN;
		}
		if (cost >= CostColor.THRESHOLD_NONE) {
			return Color.DARK_GRAY;
		}
		
		return Color.BLACK.brighter();
	}
	
}
