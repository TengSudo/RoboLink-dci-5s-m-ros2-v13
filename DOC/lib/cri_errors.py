class CRIError(Exception):
    pass

class CRIConnectionError(CRIError):
    def __init__(self, message="Not connected to iRC or connection lost."):
        self.message = message
        super().__init__(self.message)

class CRICommandTimeOutError(CRIError):
    def __init__(self, message="Time out waiting for command response."):
        self.message = message
        super().__init__(self.message)