from iocbuilder import Device
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import Asyn, AsynPort

class FINS(Device):
    Dependencies = (Asyn,)
    LibFileList = ['FINS']
    DbdFileList = ['finsUDP']
    AutoInstantiate = True

class FINSPort(AsynPort):
    pass

class FINSUDPInit(FINSPort):
    """This creates an asyn port which communicates with a FINS device over
    UDP"""
    Dependencies = (FINS,)
    
    def __init__(self, name, ip):
        self.__super.__init__(name)
        self.ip = ip

    def Initialise(self):
        print '# finsUDPInit(asyn_port, ip_addr)'    
        print 'finsUDPInit("%(asyn_name)s", "%(ip)s")' % self.__dict__
        
    ArgInfo = makeArgInfo(__init__,
        name = Simple("Asyn port name"),
        ip = Simple("IP port of FINS device"))        
        
