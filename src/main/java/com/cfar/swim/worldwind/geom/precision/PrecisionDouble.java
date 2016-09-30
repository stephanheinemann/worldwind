package com.cfar.swim.worldwind.geom.precision;

import java.math.BigDecimal;

public abstract class PrecisionDouble implements Precision {

	protected static double applyPrecision(double d, int precision) {
		BigDecimal pd = new BigDecimal(d);
		return pd.setScale(precision, BigDecimal.ROUND_HALF_UP).doubleValue();
	}
	
	public static boolean equals(double a, double b) {
		// TODO: the correct default precision should probably be a function (percentage) of the value
		return PrecisionDouble.equals(a, b, Precision.PRECISION);
	}
	
	public static boolean equals(double a, double b, int precision) {
		double pa = PrecisionDouble.applyPrecision(a, precision);
		double pb = PrecisionDouble.applyPrecision(b, precision);
		return (pa == pb);
	}
	
	public static boolean isInRange(double d, double l, double u) {
		// TODO: the correct default precision should probably be a function (percentage) of the value
		return PrecisionDouble.isInRange(d, l, u, Precision.PRECISION);
	}
	
	public static boolean isInRange(double d, double l, double u, int precision) {
		double pd = PrecisionDouble.applyPrecision(d, precision);
		double pl = PrecisionDouble.applyPrecision(l, precision);
		double pu = PrecisionDouble.applyPrecision(u, precision);
		return (pl <= pd) && (pu >= pd);
	}
	
}
