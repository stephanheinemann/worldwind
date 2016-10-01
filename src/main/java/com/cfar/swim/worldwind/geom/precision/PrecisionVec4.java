package com.cfar.swim.worldwind.geom.precision;

import gov.nasa.worldwind.geom.Vec4;

public class PrecisionVec4 extends Vec4 implements Precision {

	private int precision = Precision.PRECISION;
	private Vec4 original = null;
	
	public PrecisionVec4(Vec4 vector) {
		// TODO: the correct default precision should probably be a function (percentage) of the value
		super(
			PrecisionDouble.applyPrecision(vector.x, Precision.PRECISION),
			PrecisionDouble.applyPrecision(vector.y, Precision.PRECISION),
			PrecisionDouble.applyPrecision(vector.z, Precision.PRECISION),
			PrecisionDouble.applyPrecision(vector.w, Precision.PRECISION));
			this.original = vector;
	}
	
	public PrecisionVec4(Vec4 vector, int precision) {
		super(
			PrecisionDouble.applyPrecision(vector.x, precision),
			PrecisionDouble.applyPrecision(vector.y, precision),
			PrecisionDouble.applyPrecision(vector.z, precision),
			PrecisionDouble.applyPrecision(vector.w, precision));
		this.precision = precision;
		this.original = vector;
	}
	
	@Override
	public int getPrecision() {
		return this.precision;
	}

	@Override
	public Precision setPrecision(int precision) {
		return new PrecisionVec4(this, precision);
	}
	
	@Override
	public Vec4 getOriginal() {
		return this.original;
	}
	
}
