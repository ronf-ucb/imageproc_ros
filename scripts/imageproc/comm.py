import threading

class Comm(threading.Thread):
    """
    A Class for communication to the imageproc2.5
    """

    def subscribe(self, function):
        """Registers a callback on command received"""
        raise NotImplementedError("subscribe() not implemented")

    def sendCommand(self, status, type, data):
        """Sends a command over the interface"""
        raise NotImplementedError("sendCommand() not implemented")

