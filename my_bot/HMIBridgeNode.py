class HMIBridgeNode():

    def __init__(self):
        self.backend: SignalBackend
        self.watchdog: CommandWatchdog