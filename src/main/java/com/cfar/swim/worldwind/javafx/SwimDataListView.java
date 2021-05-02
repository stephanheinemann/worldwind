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
package com.cfar.swim.worldwind.javafx;

import java.util.ArrayList;
import java.util.List;

import com.cfar.swim.worldwind.data.DataActivationListener;
import com.cfar.swim.worldwind.util.ResourceBundleLoader;

import gov.nasa.worldwind.WorldWindow;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;

/**
 * Realizes a SWIM data list view which displays a list of SWIM data
 * identifiers.
 * 
 * @author Stephan Heinemann
 *
 */
public class SwimDataListView extends ListView<String> {

	/** the world window of this SWIM data list view */
	private WorldWindow worldWindow = null;
	
	/** the context menu of this SWIM data list view */
	private final ContextMenu contextMenu = new ContextMenu();
	
	/** the context menu item string to enable SWIM data */
	private static final String ENABLE = ResourceBundleLoader
			.getDictionaryBundle().getString("action.enable");
	
	/** the context menu item string to disable SWIM data */
	private static final String DISABLE = ResourceBundleLoader
			.getDictionaryBundle().getString("action.disable");
	
	/** the data activation listeners of this SWIM data list view */
	private List<DataActivationListener> dataActivationListeners = new ArrayList<DataActivationListener>();
	
	/**
	 * Realizes a context menu item action handler to enable or disable
	 * selected SWIM data list view items.
	 */
	private class ContextMenuItemActionHandler implements EventHandler<ActionEvent> {

		/** the SWIM data list view of this context menu item action handler */
		SwimDataListView listView = null;
		
		/**
		 * Constructs a context menu item action handler with a specified SWIM
		 * data list view.
		 * 
		 * @param listView the SWIM data list view
		 */
		public ContextMenuItemActionHandler(SwimDataListView listView) {
			this.listView = listView;
		}
		
		/**
		 * Handles action events to enable or disable selected SWIM data list
		 * view items. 
		 */
		@Override
		public void handle(ActionEvent event) {
			int index = this.listView.getSelectionModel().getSelectedIndex();
			
			if (-1 != index) {
				String identifier = this.listView.getItems().get(index);
				MenuItem menuItem = (MenuItem) event.getSource();
				
				if (menuItem.getText().equals(SwimDataListView.ENABLE)) {
					this.activate(identifier);
				} else if (menuItem.getText().equals(SwimDataListView.DISABLE)) {
					this.deactivate(identifier);
				}
				
				this.listView.worldWindow.redrawNow();
			}
		}
		
		/**
		 * Activates a selected SWIM data element for a specified SWIM data
		 * identifier by invoking registered data activation listeners .
		 * 
		 * @param identifier the SWIM data identifier
		 */
		private void activate(String identifier) {
			for (DataActivationListener dal : this.listView.dataActivationListeners) {
				dal.activate(identifier);
			}
		}
		
		/**
		 * Deactivates a selected SWIM data element for a specified SWIM data
		 * identifier by invoking registered data activation listeners .
		 * 
		 * @param identifier the SWIM data identifier
		 */
		private void deactivate(String identifier) {
			for (DataActivationListener dal : this.listView.dataActivationListeners) {
				dal.deactivate(identifier);
			}
		}
		
	}
	
	/**
	 * Constructs a SWIM data list view with a specified world window.
	 * 
	 * @param worldWindow the world window that displays SWIM data
	 */
	public SwimDataListView(WorldWindow worldWindow) {
		this.worldWindow = worldWindow;
		this.setVisible(true);
		//this.setEditable(true);
		//this.setBorder();
		
		MenuItem enableItem = new MenuItem(SwimDataListView.ENABLE);
		this.contextMenu.getItems().add(enableItem);
		MenuItem disableItem = new MenuItem(SwimDataListView.DISABLE);
		this.contextMenu.getItems().add(disableItem);
		
		for (MenuItem menuItem : this.contextMenu.getItems()) {
			menuItem.setOnAction(new ContextMenuItemActionHandler(this));
		}
		
		this.setContextMenu(this.contextMenu);
	}
	
	/**
	 * Registers a data activation listener for SWIM data to this SWIM data
	 * list view.
	 * 
	 * @param dataActivationListener the data activation listener
	 */
	public void registerDataActivationListerner(DataActivationListener dataActivationListener) {
		this.dataActivationListeners.add(dataActivationListener);
		this.getItems().addAll(dataActivationListener.getIdentifiers());
	}
	
	/**
	 * Unregisters a data activation listener for SWIM data from this SWIM data
	 * list view.
	 * 
	 * @param dataActivationListener the data activation listener
	 */
	public void unregisterDataActivationListerner(DataActivationListener dataActivationListener) {
		this.dataActivationListeners.remove(dataActivationListener);
		this.getItems().removeAll(dataActivationListener.getIdentifiers());
	}
	
}
