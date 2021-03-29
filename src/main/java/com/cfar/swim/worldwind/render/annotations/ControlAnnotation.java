/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.render.annotations;

import java.awt.Color;

import gov.nasa.worldwind.event.SelectEvent;
import gov.nasa.worldwind.render.Highlightable;
import gov.nasa.worldwindx.examples.util.ButtonAnnotation;

/**
 * Realizes a control annotation that features a primary and secondary action
 * command which is forwarded to registered action listeners as response to a
 * left and right mouse click, respectively.
 * 
 * @author Stephan Heinemann
 *
 */
public class ControlAnnotation extends ButtonAnnotation implements Highlightable {
	
	/** the primary action command of this control annotation */
	protected String primaryActionCommand = null;
	
	/** the secondary action command of this control annotation */
	protected String secondaryActionCommand = null;
	
	/** indicates whether or not this control annotation is highlighted */
	protected boolean isHighlighted;
	
	/** the default opacity for this non-highlighted control annotation */
	protected double defaultOpacity;
	
	/** the opacity for this highlighted control annotation */
	protected double highlightedOpacity;
	
	/**
	 * Constructs a new control annotation.
	 * 
	 * @see ButtonAnnotation#ButtonAnnotation()
	 */
	public ControlAnnotation() {
		this(null);
	}
	
	/**
	 * Constructs a new control annotation with a specified image source.
	 * 
	 * @param imageSource the image source
	 */
	public ControlAnnotation(Object imageSource) {
		this(imageSource, null);
	}
	
	/**
	 * Constructs a new control annotation with specified image sources for
	 * released and pressed states, respectively.
	 * 
	 * @param imageSource the image source for the released state
	 * @param pressedMaskSource the image source for the pressed state
	 */
	public ControlAnnotation(Object imageSource, Object pressedMaskSource) {
		super(imageSource, pressedMaskSource);
		this.disabledOpacity = 0.2d;
		this.defaultOpacity = 0.5d;
		this.highlightedOpacity = 1.0d;
		this.setHighlighted(false);
		this.getAttributes().setCornerRadius(10);
		this.getAttributes().setBorderColor(new Color(40, 110, 175));
		this.getAttributes().setBorderStippleFactor(1);
		this.getAttributes().setBorderWidth(0);
	}
	
	/**
	 * Gets the primary action command of this control annotation.
	 * 
	 * @return the primary action command of this control annotation
	 */
	public String getPrimaryActionCommand() {
		return this.primaryActionCommand;
	}
	
	/**
	 * Sets the primary action command of this control annotation.
	 * 
	 * @param primaryActionCommand the primary action command of this control
	 *                             annotation
	 */
	public void setPrimaryActionCommand(String primaryActionCommand) {
		this.primaryActionCommand = primaryActionCommand;
	}
	
	/**
	 * Gets the secondary action command of this control annotation.
	 * 
	 * @return the secondary action command of this control annotation
	 */
	public String getSecondaryActionCommand() {
		return this.secondaryActionCommand;
	}
	
	/**
	 * Sets the secondary action command of this control annotation.
	 * 
	 * @param secondaryActionCommand the secondary action command of this
	 *                               control annotation
	 */
	public void setSecondaryActionCommand(String secondaryActionCommand) {
		this.secondaryActionCommand = secondaryActionCommand;
	}
	
	/**
	 * Indicates whether or not a specified select event is associated with
	 * a mouse button press.
	 * 
	 * @param selectEvent the select event
	 */
	@Override
	protected boolean isButtonPressed(SelectEvent selectEvent) {
		return selectEvent.isLeftPress() || selectEvent.isRightPress();
	}
	
	/**
	 * Indicates whether or not a specified select event is associated with
	 * a mouse button click.
	 * 
	 * @param selectEvent the select event
	 */
	@Override
	protected boolean isButtonTrigger(SelectEvent selectEvent) {
		return selectEvent.isLeftClick() || selectEvent.isRightClick();
	}
	
	/**
	 * Reacts to a select event on this control annotation.
	 * 
	 * @param selectEvent the select event
	 * 
	 * @see ButtonAnnotation#selected(SelectEvent)
	 */
	@Override
	public void selected(SelectEvent selectEvent) {
		if (null != selectEvent) {
			Object selection = selectEvent.getTopObject();
			
			if (this != selection) {
				this.setHighlighted(false);
			} else if ((this == selection) && this.isEnabled()) {
			
				if (selectEvent.isHover() || selectEvent.isRollover()) {
					this.setHighlighted(true);
				}
				
				this.setPressed(this.isButtonPressed(selectEvent));
				
				if (selectEvent.isLeftPress()) {
					this.actionCommand = this.primaryActionCommand;
				} else if (selectEvent.isRightPress()) {
					this.actionCommand = this.secondaryActionCommand;
				}
				
				if (this.isButtonTrigger(selectEvent)) {
					this.onButtonPressed(selectEvent);
				}
			}
		}
	}

	/**
	 * Indicates whether or not this control annotation is highlighted.
	 * 
	 * @return true if this control annotation is highlighted, false otherwise
	 */
	@Override
	public boolean isHighlighted() {
		return this.isHighlighted;
	}

	/**
	 * Sets whether or not this control annotation is highlighted.
	 * 
	 * @param highlighted true if this control annotation is highlighted,
	 *                    false otherwise
	 */
	@Override
	public void setHighlighted(boolean highlighted) {
		this.isHighlighted = highlighted;
		if (highlighted) {
			this.getAttributes().setImageOpacity(this.highlightedOpacity);
		} else {
			this.getAttributes().setImageOpacity(this.defaultOpacity);
		}
	}
	
	/**
	 * Gets the default opacity of this non-highlighted control annotation.
	 * 
	 * @return the default opacity of this non-highlighted control annotation
	 */
	public double getDefaultOpacity() {
		return this.defaultOpacity;
	}
	
	/**
	 * Sets the default opacity of this non-highlighted control annotation.
	 * 
	 * @param defaultOpacity the default opacity of this non-highlighted
	 *                       control annotation
	 */
	public void setDefaultOpacity(double defaultOpacity) {
		this.defaultOpacity = defaultOpacity;
	}
	
	/**
	 * Gets the highlighted opacity of this control annotation.
	 * 
	 * @return the highlighted opacity of this control annotation
	 */
	public double getHighlightedOpacity() {
		return this.highlightedOpacity;
	}
	
	/**
	 * Sets the highlighted opacity of this control annotation.
	 * 
	 * @param highlightedOpacity the highlighted opacity of this control
	 *                           annotation
	 */
	public void setHighlightedOpacity(double highlightedOpacity) {
		this.highlightedOpacity = highlightedOpacity;
	}
	
	/**
	 * Frames this control annotation.
	 */
	public void frame() {
		this.getAttributes().setBorderWidth(5);
	}
	
	/**
	 * Unframes this control annotation.
	 */
	public void unframe() {
		this.getAttributes().setBorderWidth(0);
	}
	
}
