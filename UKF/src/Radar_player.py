# ---------------- TEMPLATE ---------------------------------------
# This is a template to help you start writing PythonBridge code  -
# -----------------------------------------------------------------

import rtmaps.core as rt
import rtmaps.types
from rtmaps.base_component import BaseComponent  # base class
import subprocess


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    
    # Constructor has to call the BaseComponent parent class
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor

    # Dynamic is called frequently:
    # - When loading the diagram
    # - When connecting or disconnecting a wire
    # Here you create your inputs, outputs and properties
    def Dynamic(self):
        # Adding an input called "in" of ANY type
        self.add_input("rho", rtmaps.types.ANY)  # define an input
        self.add_input("phi", rtmaps.types.ANY)  # define an input
        self.add_input("rho_dot", rtmaps.types.ANY)  # define an input
        self.add_input("timestamp", rtmaps.types.ANY)  # define an input

        # Define the output. The type is set to AUTO which means that the output will be typed automatically.
        # You don’t need to set the buffer_size, in that case it will be set automatically.
        self.add_output("x", rtmaps.types.AUTO)
        self.add_output("y", rtmaps.types.AUTO)
        self.add_output("vx", rtmaps.types.AUTO)
        self.add_output("vy", rtmaps.types.AUTO)
        
# Birth() will be called once at diagram execution startup
    def Birth(self):
        # Start the external binary
        self.process = subprocess.Popen(['external_binary'], stdout=subprocess.PIPE, stdin=subprocess.PIPE)



# Core() is called every time you have a new inputs available, depending on your chosen reading policy
    def Core(self):
        # Just copy the input to the output here
        # Communicate with the process through its standard input and output
        stdout, stderr = process.communicate(input=[
                                             self.inputs["rho"],
                                             self.inputs["phi"],
                                             self.inputs["rho_dot"],
                                             self.inputs["timestamp"]])
        
        out = np.fromstring(output)

        self.write("x", out[0])
        self.write("y", out[1])
        self.write("vx", out[2])
        self.write("vx", out[3])

# Death() will be called once at diagram execution shutdown
    def Death(self):
        print("Passing through Death()")
