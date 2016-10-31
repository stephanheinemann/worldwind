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

public class DepictionTest {

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
