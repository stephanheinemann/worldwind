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
package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertNotNull;

import java.io.IOException;
import java.net.URISyntaxException;
import java.net.URL;

import org.junit.Test;

import gov.nasa.worldwind.Configuration;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.avlist.AVList;
import gov.nasa.worldwind.avlist.AVListImpl;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525IconRetriever;

/**
 * Performs depiction tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class DepictionTest {

	/**
	 * Tests military standard 2525 symbol resources.
	 * 
	 * @throws IOException if symbols cannot be loaded
	 * @throws URISyntaxException if the symbol resource URI is incorrect
	 */
	@Test
	public void testMilStd2525() throws IOException, URISyntaxException {
		String path = Configuration.getStringValue(AVKey.MIL_STD_2525_ICON_RETRIEVER_PATH);
		assertNotNull(path);
		
		URL url = ClassLoader.getSystemResource("milstd2525");
		assertNotNull(url);
		
		MilStd2525IconRetriever ir = new MilStd2525IconRetriever(url.toString());
		AVList params = new AVListImpl();
		params.setValue("SymbologyConstants.SHOW_ICON", true);
		params.setValue("SymbologyConstants.SHOW_FRAME", true);
		params.setValue("SymbologyConstants.SHOW_FILL", true);
		params.setValue("AVKey.COLOR", java.awt.Color.CYAN);
		assertNotNull(ir.createIcon("SFAPCF---------", params));
	}

}
