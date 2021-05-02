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
package com.cfar.swim.worldwind.session;

import java.util.HashSet;
import java.util.Optional;

/**
 * Realizes a session manager.
 * 
 * @author Stephan Heinemann
 *
 */
public class SessionManager {

	/** the singleton session manager instance */
	private static SessionManager manager = new SessionManager();
	
	/** the managed session of this session manager */
	private HashSet<Session> sessions = new HashSet<Session>();
	
	/**
	 * Constructs the singleton session manager instance.
	 */
	private SessionManager() {
	}
	
	/**
	 * Gets the singleton session manager instance.
	 * 
	 * @return the singleton session manager instance
	 */
	public static SessionManager getInstance() {
		return SessionManager.manager;
	}
	
	/**
	 * Gets a session with a specified identifier from this session manager
	 * if present.
	 * 
	 * @param id the session identifier
	 * 
	 * @return the identified session if present, null otherwise
	 */
	public Session getSession(String id) {
		Session session = null;
		Optional<Session> optSession = this.sessions.stream().filter(s -> s.getId().equals(id)).findFirst();
		
		if (optSession.isPresent()) {
			session = optSession.get();
		}
		
		return session;
	}
	
	/**
	 * Adds a session to this session manager if not present.
	 * 
	 * @param session the session to be added
	 */
	public void addSession(Session session) {
		this.sessions.add(session);
	}

}
