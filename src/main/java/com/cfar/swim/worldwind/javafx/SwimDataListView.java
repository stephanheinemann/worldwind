package com.cfar.swim.worldwind.javafx;

import java.util.ArrayList;
import java.util.List;

import com.cfar.swim.worldwind.data.DataActivationListener;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;

public class SwimDataListView extends ListView<String> {

	private final ContextMenu contextMenu = new ContextMenu();
	private static final String ENABLE = "Enable";
	private static final String DISABLE = "Disable";
	
	private List<DataActivationListener> dataActivationListeners = new ArrayList<DataActivationListener>();
	
	private class ContextMenuItemActionHandler implements EventHandler<ActionEvent> {

		SwimDataListView listView = null;
		
		public ContextMenuItemActionHandler(SwimDataListView listView) {
			this.listView = listView;
		}
		
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
			}
		}
		
		private void activate(String identifier) {
			for (DataActivationListener dal : this.listView.dataActivationListeners) {
				if (dal.getIdentifier().equals(identifier)) {
					dal.activate();
				}
			}
		}
		
		private void deactivate(String identifier) {
			for (DataActivationListener dal : this.listView.dataActivationListeners) {
				if (dal.getIdentifier().equals(identifier)) {
					dal.deactivate();
				}
			}
		}
		
	}
	
	public SwimDataListView() {
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
	
	public void registerDataActivationListerner(DataActivationListener dataActivationListener) {
		this.dataActivationListeners.add(dataActivationListener);
	}
	
	public void unregisterDataActivationListerner(DataActivationListener dataActivationListener) {
		this.dataActivationListeners.remove(dataActivationListener);
	}
	
}
