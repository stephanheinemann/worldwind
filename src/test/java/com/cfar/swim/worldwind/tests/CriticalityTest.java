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

import java.net.URI;

import org.junit.Test;

import com.cfar.swim.worldwind.managing.Difficulty;
import com.cfar.swim.worldwind.managing.FeatureTuning;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.KnowledgeBase;
import com.cfar.swim.worldwind.managing.Severity;
import com.cfar.swim.worldwind.managing.Tuning;
import com.cfar.swim.worldwind.registries.managers.AbstractManagerProperties;

/**
 * Performs criticality tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class CriticalityTest {
	
	@Test
	public void testDifficulty() {
		int lowCount = 0, modCount = 0, subCount = 0, sevCount =0, crtCount = 0;
		
		KnowledgeBase knowledgeBase = new KnowledgeBase();
		knowledgeBase.load(URI.create(AbstractManagerProperties.KNOWLEDGE_BASE_RESOURCE));
		//System.out.println(knowledgeBase.toString());
		
		for (Tuning<?>  tuning : knowledgeBase.getReputation().keySet()) {
			if (tuning instanceof FeatureTuning) {
				Features features = ((FeatureTuning<?>) tuning).getFeatures();
				Difficulty low = Difficulty.createLow(features);
				Difficulty moderate = Difficulty.createModerate(features);
				Difficulty substantial = Difficulty.createSubstantial(features);
				Difficulty severe = Difficulty.createSevere(features);
				Difficulty critical = Difficulty.createCritical(features);
				if (low.covers(features)) lowCount++;
				if (moderate.covers(features)) modCount++;
				if (substantial.covers(features)) subCount++;
				if (severe.covers(features)) sevCount++;
				if (critical.covers(features)) crtCount++;
			}
		}
		System.out.println("difficulty: low = " + lowCount
				+ ", moderate = " + modCount
				+ ", substantial = " + subCount
				+ ", severe = " + sevCount
				+ ", critical = " + crtCount);
	}
	
	@Test
	public void testSeverity() {
		int lowCount = 0, modCount = 0, subCount = 0, sevCount =0, crtCount = 0;
		
		KnowledgeBase knowledgeBase = new KnowledgeBase();
		knowledgeBase.load(URI.create(AbstractManagerProperties.KNOWLEDGE_BASE_RESOURCE));
		//System.out.println(knowledgeBase.toString());
		
		for (Tuning<?>  tuning : knowledgeBase.getReputation().keySet()) {
			if (tuning instanceof FeatureTuning) {
				Features features = ((FeatureTuning<?>) tuning).getFeatures();
				Severity low = Severity.createLow(features);
				Severity moderate = Severity.createModerate(features);
				Severity substantial = Severity.createSubstantial(features);
				Severity severe = Severity.createSevere(features);
				Severity critical = Severity.createCritical(features);
				if (low.covers(features)) lowCount++;
				if (moderate.covers(features)) modCount++;
				if (substantial.covers(features)) subCount++;
				if (severe.covers(features)) sevCount++;
				if (critical.covers(features)) crtCount++;
			}
		}
		System.out.println("severity: low = " + lowCount
				+ ", moderate = " + modCount
				+ ", substantial = " + subCount
				+ ", severe = " + sevCount
				+ ", critical = " + crtCount);
	}
	
}
