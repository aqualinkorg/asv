from conans import ConanFile, CMake, tools, RunEnvironment
import os

class ASVNanoConan( ConanFile ):
    name            = "asv_nano_ws"
    version         = "1.0.0"
    author          = "developer@aqualink.org"
    description     = ""
    license         = ""

    settings        = "os", "compiler", "arch", "build_type"
    options         = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    generators      = "cmake"

    requires =  (
                    ( "Catch2/2.5.0@catchorg/stable" ),
                    ( "spdlog/1.3.1@bincrafters/stable" ),
                    ( "boost/1.71.0@conan/stable" ),
                    ( "mavchannel/1.0.0@aqualink/stable" ),
                    ( "libjpeg-turbo/2.0.4@" )
                )