package com.cfar.swim.worldwind.managing;

import java.time.Duration;

/**
 * Realizes a duration quantity.
 * 
 * @author Stephan Heinemann
 *
 * @see Quantity
 */
public class DurationQuantity implements Quantity {
	
	/** the quantity of this duration quantity */
	private final Duration quantity;
	
	/**
	 * Constructs a new duration quantity based on a duration.
	 * 
	 * @param quantity the duration
	 * 
	 * @throws IllegalArgumentException if the quantity is invalid
	 */
	public DurationQuantity(Duration quantity) {
		if ((null == quantity) || ((null != quantity)
				&& (quantity.isNegative() || quantity.isZero()))) {
			throw new IllegalArgumentException("quantity is invalid");
		}
		this.quantity = quantity;
	}
	
	/**
	 * Gets the duration quantity in seconds.
	 * 
	 * @return the duration quantity in seconds
	 * 
	 * @see Quantity#get()
	 */
	@Override
	public double get() {
		return quantity.getSeconds() + (quantity.getNano() * 10E-9d);
	}
	
}
