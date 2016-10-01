package com.cfar.swim.worldwind.geom.precision;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;

public class PrecisionPosition extends Position implements Precision {

	private int precision = Precision.PRECISION;
	private Position original = null;
	
	public PrecisionPosition(Position position) {
		// TODO: the correct default precision should probably be a function (percentage) of the value
		super(
			Angle.fromDegrees(PrecisionDouble.applyPrecision(position.latitude.degrees, Precision.PRECISION)),
			Angle.fromDegrees(PrecisionDouble.applyPrecision(position.longitude.degrees, Precision.PRECISION)),
			PrecisionDouble.applyPrecision(position.elevation, Precision.PRECISION));
		this.original = position;
	}

	public PrecisionPosition(Position position, int precision) {
		super(
			Angle.fromDegrees(PrecisionDouble.applyPrecision(position.latitude.degrees, precision)),
			Angle.fromDegrees(PrecisionDouble.applyPrecision(position.longitude.degrees, precision)),
			PrecisionDouble.applyPrecision(position.elevation, precision));
		this.precision = precision;
		this.original = position;
	}
		
	@Override
	public int getPrecision() {
		return this.precision;
	}

	@Override
	public Precision setPrecision(int precision) {
		return new PrecisionPosition(this, precision);
	}
	
	@Override
	public Position getOriginal() {
		return this.original;
	}
	
}
