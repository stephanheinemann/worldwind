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
import java.awt.Dimension;

import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.GlobeAnnotation;

/**
 * Realizes an annotation for a depiction.
 * 
 * @author Stephan Heinemann
 *
 */
public class DepictionAnnotation extends GlobeAnnotation {

	/**
	 * Constructs a new depiction annotation with a specified text at a
	 * specified position.
	 * 
	 * @param text the text of this depiction annotation
	 * @param position the position of this depiction annotation in globe
	 *        coordinates
	 * 
	 * @see GlobeAnnotation#GlobeAnnotation(String, Position)
	 */
	public DepictionAnnotation(String text, Position position) {
		super(text, position);
		this.getAttributes().setTextColor(Color.LIGHT_GRAY);
		this.getAttributes().setBackgroundColor(new Color(0, 0, 0, 0));
		this.getAttributes().setTextAlign(AVKey.CENTER);
		this.getAttributes().setLeaderGapWidth(3);
	}
	
	/**
	 * Constructs a new depiction annotation with a specified image source
	 * and text at a specified position.
	 * 
	 * @param imageSource the image source of this depiction annotation
	 * @param text the text of this depiction annotation
	 * @param position the position of this depiction annotation in globe
	 *        coordinates
	 */
	public DepictionAnnotation(Object imageSource, String text, Position position) {
		super(text, position);
		this.getAttributes().setImageSource(imageSource);
		this.getAttributes().setImageScale(1d);
		this.getAttributes().setImageOpacity(1d);
		this.getAttributes().setImageRepeat(AVKey.REPEAT_NONE);
		this.getAttributes().setTextColor(Color.LIGHT_GRAY);
		this.getAttributes().setBackgroundColor(new Color(0, 0, 0, 0));
		this.getAttributes().setAdjustWidthToText(AVKey.SIZE_FIXED);
		this.getAttributes().setSize(new Dimension(64, 64));
		this.getAttributes().setTextAlign(AVKey.CENTER);
		this.getAttributes().setLeaderGapWidth(3);
	}
	
	/**
	 * Constructs a new depiction annotation with a specified image source,
	 * size and text at a specified position.
	 * 
	 * @param imageSource the image source of this depiction annotation
	 * @param dim the size of this depiction annotation
	 * @param text the text of this depiction annotation
	 * @param position the position of this depiction annotation in globe
	 *        coordinates
	 */
	public DepictionAnnotation(Object imageSource, Dimension dim, String text, Position position) {
		super(text, position);
		this.getAttributes().setImageSource(imageSource);
		this.getAttributes().setImageScale(1d);
		this.getAttributes().setImageOpacity(1d);
		this.getAttributes().setImageRepeat(AVKey.REPEAT_NONE);
		this.getAttributes().setTextColor(Color.LIGHT_GRAY);
		this.getAttributes().setBackgroundColor(new Color(0, 0, 0, 0));
		this.getAttributes().setAdjustWidthToText(AVKey.SIZE_FIXED);
		this.getAttributes().setSize(dim);
		this.getAttributes().setTextAlign(AVKey.CENTER);
		this.getAttributes().setLeaderGapWidth(3);
	}

}
