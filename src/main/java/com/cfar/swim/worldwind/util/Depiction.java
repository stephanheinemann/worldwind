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
package com.cfar.swim.worldwind.util;

import gov.nasa.worldwind.WorldWind;
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

/* TODO:
 * Annotations and depictions should be instantiated only once per message and moved during time updates
 * The same could apply to any entire obstacle being part of the same message
 */

public class Depiction implements Renderable {

	private Renderable depiction = null;
	private static final TacticalSymbolAttributes symbolAttributes = new BasicTacticalSymbolAttributes();
	private static final TacticalGraphicAttributes graphicAttributes = new BasicTacticalGraphicAttributes();
	
	public Depiction(TacticalGraphic depiction) {
		this.depiction = depiction;
		depiction.setAttributes(Depiction.graphicAttributes);
		depiction.getAttributes().setScale(0.5);
		depiction.getAttributes().setTextModifierMaterial(Material.WHITE);
		depiction.setVisible(false);
	}
	
	public Depiction(TacticalSymbol depiction) {
		this.depiction = depiction;
		depiction.setAttributes(Depiction.symbolAttributes);
		depiction.getAttributes().setScale(0.5);
		depiction.getAttributes().setTextModifierMaterial(Material.WHITE);
		depiction.setAltitudeMode(WorldWind.ABSOLUTE);
		depiction.setVisible(false);
	}
	
	@Override
	public void render(DrawContext dc) {
		this.depiction.render(dc);
	}
	
	public void setVisible(boolean visible) {
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).setVisible(visible);
		}
		if (this.depiction instanceof TacticalSymbol) {
			((TacticalSymbol) this.depiction).setVisible(visible);
		}
	}
	
	public void setDesignation(String designation) {
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).setText(designation);
		}
		if (this.depiction instanceof TacticalSymbol) {
			((TacticalSymbol) this.depiction).setModifier(SymbologyConstants.UNIQUE_DESIGNATION, designation);
		}
	}
	
	public void setModifier(String modifier, Object value) {
		if (this.depiction instanceof TacticalGraphic) {
			((TacticalGraphic) this.depiction).setModifier(modifier, value);
		}
		if (this.depiction instanceof TacticalSymbol) {
			((TacticalSymbol) this.depiction).setModifier(modifier, value);
		}
	}
	
	public Object getModifier(String modifier) {
		Object value = null;
		
		if (this.depiction instanceof TacticalGraphic) {
			value = ((TacticalGraphic) this.depiction).getModifier(modifier);
		}
		if (this.depiction instanceof TacticalSymbol) {
			value = ((TacticalSymbol) this.depiction).getModifier(modifier);
		}
		
		return value;
	} 

}
