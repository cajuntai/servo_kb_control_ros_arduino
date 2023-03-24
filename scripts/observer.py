#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  8 18:30:46 2023

@author: cajun
"""


import logging
import traceback

class HasObservers(object):
    def __init__(self):
        logging.basicConfig()
        self._logger = logging.getLogger(__name__)

        self._event_observers = {}
        self._event_cache = {}
        self._exclusive_classes = []

    def add_event_listener(self, event_type, observer):
        listeners_for_event = self._event_observers.get(event_type)
        if listeners_for_event is None:
            listeners_for_event = []
            self._event_observers[event_type] = listeners_for_event
        if observer not in listeners_for_event:
            listeners_for_event.append(observer)
    
    def remove_event_listener(self, event_type, observer):
        listeners_for_event = self._event_observers.get(event_type)
        if listeners_for_event is not None:
            listeners_for_event.remove(observer)
            if len(listeners_for_event) == 0:
                del self._event_observers[event_type]
        print(self._event_observers)
    
    def notify_event_observers(self, event_type, event):   # Changed this to *event (commenting in case got bug)
        """
        :param String event_type: The event type that has been received.
        :param event: The event that has been received. (The data)
        """
        self.notify(event_type, event)
        self.notify("*", event)
    
    def notify(self, event_type, event):
        for fn in self._event_observers.get(event_type, []):
            try:
                fn(self, event_type, event)
            except Exception as e:
                print(f"Exeception occured in {self.__module__}: ", e)
                print("\nTraceback:\n", traceback.format_exc())
                self._logger.exception('Exception in event handler for %s' % event_type, exc_info=True)
                
    def on_event(self, event_type):
        """
        :param String event_type: The event type to watch (or '*' to watch all events).
        :param observer: The callback to invoke when an event of the specified type is received.
        """
        def decorator(fn):
            if isinstance(event_type, list):  # Register multi-condition callbacks in one decorator
                for n in event_type:
                    self.add_event_listener(n, fn)
            else:
                self.add_event_listener(event_type, fn)
        return decorator


if __name__ == "__main__":
    
    obs = HasObservers()
    
    @obs.on_event('A')
    def event_a(self, event, event_type):
        print('Event A...')
        
    obs.notify_event_observers('A')
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    