package com.cfar.swim.worldwind.render.annotations;

import java.awt.Color;
import java.awt.Dimension;

import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Annotation;
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
		this.getAttributes().setImageRepeat(Annotation.IMAGE_REPEAT_NONE);
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
		this.getAttributes().setImageRepeat(Annotation.IMAGE_REPEAT_NONE);
		this.getAttributes().setTextColor(Color.LIGHT_GRAY);
		this.getAttributes().setBackgroundColor(new Color(0, 0, 0, 0));
		this.getAttributes().setAdjustWidthToText(AVKey.SIZE_FIXED);
		this.getAttributes().setSize(dim);
		this.getAttributes().setTextAlign(AVKey.CENTER);
		this.getAttributes().setLeaderGapWidth(3);
	}

}
