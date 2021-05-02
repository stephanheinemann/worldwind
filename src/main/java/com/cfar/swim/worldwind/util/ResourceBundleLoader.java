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

import java.util.Locale;
import java.util.ResourceBundle;

import gov.nasa.worldwind.Configuration;

/**
 * Realizes a resource bundle loader.
 * 
 * @author Stephan Heinemann
 *
 */
public final class ResourceBundleLoader {

	/** the locale configuration key of this resource bundle loader */
	private static final String LOCALE_KEY = "com.cfar.swim.worldwind.avkey.Locale";
	
	/** the dictionary bundle */
	private static final String DICTIONARY_BUNDLE = "com.cfar.swim.worldwind.dictionaries.Dictionary";
	
	/**
	 * Gets the dictionary bundle for the configured locale.
	 * 
	 * @return the dictionary bundle for the configured locale
	 */
	public static ResourceBundle getDictionaryBundle() {
		Locale locale = new Locale(Configuration.getStringValue(ResourceBundleLoader.LOCALE_KEY));
		return ResourceBundle.getBundle(ResourceBundleLoader.DICTIONARY_BUNDLE, locale);
	}
	
	/**
	 * Gets the dictionary bundle for a specified locale.
	 * 
	 * @param locale the dictionary locale
	 * 
	 * @return the dictionary bundle for the specified locale
	 */
	public static ResourceBundle getDictionaryBundle(Locale locale) {
		return ResourceBundle.getBundle(ResourceBundleLoader.DICTIONARY_BUNDLE, locale);
	}
	
}
