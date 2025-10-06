#!/usr/bin/env python3
from threading import Timer

class IdleTimer:
    def __init__(self, interval, function = None):
        if function is None:
            self.function = self.dummyfunction
        else:
            self.function = function
        self.interval = interval
        self._triggered = False
        self.timer = Timer(self.interval, self.callback)
        self.timer.start()

    def dummyfunction(self):
        pass

    def reset(self):
        self.timer.cancel()
        self.timer = Timer(self.interval, self.callback)
        self.timer.start()
        self._triggered = False
    
    def istriggered(self): return self._triggered
    def callback(self): 
        self.function()
        self._triggered = True

class RepeatUntilTrue:
    def __init__(self, interval, executefunction, testfunction):
        self.interval = interval
        self.testfunction = testfunction
        self.executefunction = executefunction
        self.gentimer()
    def gentimer(self):
        self.timer = Timer(self.interval, self.callback)
        self.timer.start() 
    def callback(self): 
        if self.testfunction() == False:
            self.executefunction()
            self.gentimer()
            
class RepeatUntilTrueWrapup(RepeatUntilTrue):
    def __init__(self, interval, executefunction, testfunction, wrapupfunction):
        self.interval = interval
        self.testfunction = testfunction
        self.executefunction = executefunction
        self.wrapupfunction = wrapupfunction
        self.gentimer()
    def callback(self): 
        if self.testfunction() == False:
            self.executefunction()
            self.gentimer()
        else: # do only once, if repeat is true
            self.wrapupfunction()