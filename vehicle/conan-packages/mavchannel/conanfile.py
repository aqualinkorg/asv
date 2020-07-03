from conans import ConanFile, CMake, tools

import os
import subprocess

class MavchannelConan(ConanFile):
    name            = "mavchannel"
    version         = "1.0.0"
    author          = "charles@missionrobotics.us"
    description     = "Utility library for sending and receiving mavlink messages over serial ports"
    license         = "MIT"

    settings        = "os", "compiler", "build_type", "arch"
    options         = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True } 
    
    requires        =   ( 
                            ( "readerwriterqueue/1.0.1@aqualink/stable" ),
                            ( "concurrentqueue/1.0.0@aqualink/stable" ),
                            ( "serial/1.2.1@aqualink/stable" ),
                            ( "mavlink2/1.0.0@aqualink/stable" )
                        )

    exports = "include/*", "src/*", "CMakeLists.txt"
    generators = "cmake"

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        # Copy includes
        self.copy("*.hpp", dst="include", src="include")

        # Copy libs
        self.copy("*.a", dst="lib", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ['mavchannel']
