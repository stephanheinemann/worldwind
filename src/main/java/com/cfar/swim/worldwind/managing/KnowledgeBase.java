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
package com.cfar.swim.worldwind.managing;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Realizes a knowledge base of an autonomic manager.
 * 
 * @author Stephan Heinemann
 *
 */
public class KnowledgeBase implements Serializable {
	
	/** the default serial identification of this knowledge base */
	private static final long serialVersionUID = 1L;
	
	/** the performance reputation of this knowledge base */
	private Reputation reputation = new Reputation();
	
	// reputations and model (fit model with reputations)
	// load and save knowledge to database
	
	/**
	 * Gets the reputation of this knowledge base.
	 * 
	 * @return the reputation of this knowledge base
	 */
	public Reputation getReputation() {
		return this.reputation;
	}
	
	/**
	 * Sets the reputation of this knowledge base.
	 * 
	 * @param reputation the reputation of this knowledge base
	 * 
	 * @throws IllegalArgumentException if reputation is invalid
	 */
	public void setReputation(Reputation reputation) {
		if (null == reputation) {
			throw new IllegalArgumentException("invalid reputation");
		}
		this.reputation = reputation;
	}
	
	/**
	 * Loads this knowledge base.
	 * 
	 * @param uri the URI to load this knowledge base from
	 * 
	 * @return true if this knowledge base has been loaded, false otherwise
	 */
	public boolean load(URI uri) {
		boolean isLoaded = false;
		
		try {
			FileInputStream fis = new FileInputStream(new File(uri));
			ObjectInputStream ois = new ObjectInputStream(fis);
			this.setReputation((Reputation) ois.readObject());
			ois.close();
			isLoaded = true;
		} catch (IOException | ClassNotFoundException e) {
			e.printStackTrace();
		}
		
		return isLoaded;
	}
	
	/**
	 * Saves this knowledge base.
	 * 
	 * @param uri the URI to save this knowledge base to
	 * 
	 * @return true if this knowledge base has been save, false otherwise
	 */
	public boolean save(URI uri) {
		boolean isSaved = false;
		
		try {
			Files.createDirectories(Path.of(uri).getParent());
			FileOutputStream fos = new FileOutputStream(new File(uri));
			ObjectOutputStream oos = new ObjectOutputStream(fos);
			oos.writeObject(this.getReputation());
			oos.flush();
			oos.close();
			isSaved = true;
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return isSaved;
	}
	
	/**
	 * Gets the string representation of this knowledge base.
	 * 
	 * @return the string representation of this knowledge base
	 * 
	 * @see Object#toString()
	 */
	@Override
	public String toString() {
		return this.getReputation().toString();
	}
	
}
