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
package com.cfar.swim.worldwind.util;

import com.cfar.swim.worldwind.render.annotations.DepictionAnnotation;

import gov.nasa.worldwind.Movable;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.symbology.BasicTacticalGraphicAttributes;
import gov.nasa.worldwind.symbology.BasicTacticalSymbolAttributes;
import gov.nasa.worldwind.symbology.SymbologyConstants;
import gov.nasa.worldwind.symbology.TacticalGraphic;
import gov.nasa.worldwind.symbology.TacticalGraphicAttributes;
import gov.nasa.worldwind.symbology.TacticalSymbol;
import gov.nasa.worldwind.symbology.TacticalSymbolAttributes;

// TODO: Annotations and depictions should be instantiated only once per
// message and moved during time updates. The same could apply to any entire
// obstacle being part of the same message.
// TODO: A depiction could aggregate several symbols / geometries and render
// all of them. For example, a waypoint can have a ground POI symbol and an
// air WPT symbol at altitude and a text annotation (times, costs).

/**
 * Realizes an annotatable depiction that aggregates either a tactical symbol
 * or graphic.
 * 
 * @author Stephan Heinemann
 *
 */
public class Depiction implements Renderable, Movable {

	/** the tactical symbol or graphic of this depiction */
	private Renderable depiction = null;
	private static final TacticalSymbolAttributes symbolAttributes = new BasicTacticalSymbolAttributes();
	private static final TacticalGraphicAttributes graphicAttributes = new BasicTacticalGraphicAttributes();
	
	/** the annotation of this depiction */
	private DepictionAnnotation annotation = null;
	
	/**
	 * Constructs a new depiction from a tactical graphic.
	 * 
	 * @param depiction the tactical graphic
	 */
	public Depiction(TacticalGraphic depiction) {
		this.depiction = depiction;
		depiction.setAttributes(Depiction.graphicAttributes);
		depiction.getAttributes().setScale(0.5);
		depiction.getAttributes().setTextModifierMaterial(Material.WHITE);
		depiction.setVisible(false);
	}
	
	/**
	 * Constructs a new depiction from a tactical symbol.
	 * 
	 * @param depiction the tactical symbol
	 */
	public Depiction(TacticalSymbol depiction) {
		this.depiction = depiction;
		depiction.setAttributes(Depiction.symbolAttributes);
		depiction.getAttributes().setScale(0.5);
		depiction.getAttributes().setTextModifierMaterial(Material.WHITE);
		depiction.setAltitudeMode(WorldWind.ABSOLUTE);
		depiction.setVisible(false);
	}
	
	/**
	 * Gets the tactical depiction of this depiction.
	 * 
	 * @return the tactical depiction of this depiction
	 */
	public Renderable getTacticalDepiction() {
		return this.depiction;
	}
	
	/**
	 * Renders this depiction.
	 * 
	 * @see Renderable#render(DrawContext)
	 */
	@Override
	public void render(DrawContext dc) {
		this.depiction.render(dc);
		if (null != this.annotation) {
			this.annotation.render(dc);
		}
	}
	
	/**
	 * Sets whether or not this depiction is visible.
	 * 
	 * @param visible the visibility state to be set
	 */
	public void setVisible(boolean visible) {
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).setVisible(visible);
		} else if (this.depiction instanceof TacticalSymbol) {
			((TacticalSymbol) this.depiction).setVisible(visible);
		}
		if (null != this.annotation) {
			this.annotation.getAttributes().setVisible(visible);
		}
	}
	
	/**
	 * Sets the designation of this depiction.
	 * 
	 * @param designation the designation to be set
	 */
	public void setDesignation(String designation) {
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).setText(designation);
		} else if (this.depiction instanceof TacticalSymbol) {
			((TacticalSymbol) this.depiction).setModifier(SymbologyConstants.UNIQUE_DESIGNATION, designation);
		}
	}
	
	/**
	 * Sets a modifier value of this depiction.
	 * 
	 * @param modifier the modifier
	 * @param value the value of the modifier
	 * 
	 * @see TacticalGraphic#setModifier(String, Object)
	 * @see TacticalSymbol#setModifier(String, Object)
	 */
	public void setModifier(String modifier, Object value) {
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).setModifier(modifier, value);
		} else if (this.depiction instanceof TacticalSymbol) {
			((TacticalSymbol) this.depiction).setModifier(modifier, value);
		}
	}
	
	/**
	 * Gets a modifier value of this depiction.
	 * 
	 * @param modifier the modifier
	 * @return the value of the modifier
	 * 
	 * @see TacticalGraphic#getModifier(String)
	 * @see TacticalSymbol#getModifier(String)
	 */
	public Object getModifier(String modifier) {
		Object value = null;
		
		if (this.depiction instanceof TacticalGraphic) {
			value = ((TacticalGraphic) this.depiction).getModifier(modifier);
		} else if (this.depiction instanceof TacticalSymbol) {
			value = ((TacticalSymbol) this.depiction).getModifier(modifier);
		}
		
		return value;
	}
	
	/**
	 * Gets the annotation of this depiction.
	 * 
	 * @return the annotation of this depiction
	 */
	public DepictionAnnotation getAnnotation() {
		return this.annotation;
	}
	
	/**
	 * Set the annotation of this depiction.
	 * 
	 * @param annotation the annotation to be set
	 */
	public void setAnnotation(DepictionAnnotation annotation) {
		this.annotation = annotation;
	}
	
	/**
	 * Indicates whether or not this depiction has an annotation.
	 * 
	 * @return true if this depiction has an annotation, false otherwise
	 */
	public boolean hasAnnotation() {
		return (null != this.annotation);
	}
	
	/**
	 * Gets the symbol identifier of this depiction.
	 * 
	 * @return the symbol identifier of this depiction
	 */
	public String getSymbolIdentifier() {
		String sidc = null;
		
		if (this.depiction instanceof TacticalGraphic) {
			sidc = ((TacticalGraphic) this.depiction).getIdentifier();
		} else if (this.depiction instanceof TacticalSymbol) {
			sidc = ((TacticalSymbol) this.depiction).getIdentifier();
		}
		
		return sidc;
	}
	
	/**
	 * Gets the reference position of this depiction.
	 * 
	 * @return the reference position of this depiction
	 * 
	 * @see Movable#getReferencePosition()
	 */
	@Override
	public Position getReferencePosition() {
		Position referencePosition = null;
		
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).getReferencePosition();
		} else if (this.depiction instanceof TacticalSymbol) {
			((TacticalSymbol) this.depiction).getPosition();
		}
		
		return referencePosition;
	}
	
	/**
	 * Moves this depiction to a specified position.
	 * 
	 * @param position the position to move this depiction to
	 * 
	 * @see Movable#moveTo(Position)
	 */
	@Override
	public void moveTo(Position position) {
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).moveTo(position);
		} else if (this.depiction instanceof TacticalSymbol) {
			((TacticalSymbol) this.depiction).setPosition(position);
		}
		
		if (null != this.annotation) {
			this.annotation.moveTo(position);
		}
	}
	
	/**
	 * Moves this depiction by adding a specified position.
	 * 
	 * @param position the position to be added to this depiction's position
	 * 
	 * @see Movable#move(Position)
	 */
	@Override
	public void move(Position position) {
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).move(position);
		} else if (this.depiction instanceof TacticalSymbol) {
			TacticalSymbol symbol = ((TacticalSymbol) this.depiction);
			symbol.setPosition(symbol.getPosition().add(position));
		}
		
		if (null != this.annotation) {
			this.annotation.move(position);
		}
	}
	
}
