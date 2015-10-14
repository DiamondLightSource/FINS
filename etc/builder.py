from iocbuilder import AutoSubstitution, Substitution, ModuleBase
from iocbuilder import Device, SetSimulation
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import Asyn, AsynPort

class FINS(Device):
    Dependencies = (Asyn,)
    LibFileList = ['FINS']
    DbdFileList = ['FINS']
    AutoInstantiate = True

class FINSPort(AsynPort):
    pass

class FINSUDPInit(FINSPort):
    """This creates an asyn port which communicates with a FINS device over
    UDP"""
    Dependencies = (FINS,)
    
    def __init__(self, ip, name, simulation=None, **args):
        self.__super.__init__(name)
        self.ip = ip
        self.simulation = simulation

    def Initialise(self):
        print '# finsUDPInit(asyn_port, ip_addr)'    
        print 'finsUDPInit("%(asyn_name)s", "%(ip)s")' % self.__dict__
        
    ArgInfo = makeArgInfo(__init__,    
        ip = Simple("IP port of FINS device"),
        name = Simple("Asyn port name"),
        simulation = Simple("IP port of simulation device"))     
    
def FINSUDPInitSim(ip, name, simulation=None, FINSUDPInit=FINSUDPInit, **kwargs):
    if simulation:
        return FINSUDPInit(simulation, name, **kwargs)

SetSimulation(FINSUDPInit, FINSUDPInitSim)

class FINSHostlink(FINSPort):
    """This creates a port that will send FINS command in a Hostlink wrapper
    to a PLC over a serial port."""
    Dependencies = (FINS,)
    
    def __init__(self, asyn_port, name, **args):
        self.__super.__init__(name)
        self.asyn_port = asyn_port

    def Initialise(self):
        print '# HostlinkInterposeInit(asyn_port)'    
        print 'HostlinkInterposeInit("%(asyn_port)s")' % self.__dict__    
        print '# finsDEVInit(FINS_port_name, asyn_port)'    
        print 'finsDEVInit("%(asyn_name)s", "%(asyn_port)s")' % self.__dict__
        
    ArgInfo = makeArgInfo(__init__,    
        asyn_port = Ident("Asyn port for serial comms (possibly over terminal server)", AsynPort),
        name = Simple("Asyn port name created by FINS driver"))     
    
class FINSTemplate(AutoSubstitution):
    TemplateFile = 'FINS.template'
