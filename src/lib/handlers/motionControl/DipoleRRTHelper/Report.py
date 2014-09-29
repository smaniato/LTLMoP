
import cPickle as pickle

class ReportLtlmop:
    def __init__(self, savePath):
        self.savePath = savePath
        self.constrainedRegions = None
        
    @classmethod
    def loadFromFile(cls, savePath):
        report = cls(savePath)
        with open(savePath, "rb") as f:
            pass
        return report
        
    def save(self):
        with open(self.savePath, 'wb') as f:
            pass
    