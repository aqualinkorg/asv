from conans import ConanFile, CMake, tools
import os


class ReaderWriterQueueConan(ConanFile):
    name = "readerwriterqueue"
    version = "1.0.1"
    settings = "os", "compiler", "arch", "build_type"
    description = "A single-producer, single-consumer lock-free queue for C++"
    license = ("Simplified BSD", "zlib")
    repo_url = "https://github.com/cameron314/readerwriterqueue"
    author = "Aqualink Developer <developer@aqualink.org>"

    def source(self):
        tools.get("%s/archive/v%s.zip" % (self.repo_url, self.version))

    def package(self):
        self.copy( "LICENSE.md",             src="readerwriterqueue-%s" % self.version )
        self.copy( "atomicops.h",            src="readerwriterqueue-%s" % self.version, dst="include/readerwriterqueue" )
        self.copy( "readerwriterqueue.h",    src="readerwriterqueue-%s" % self.version, dst="include/readerwriterqueue" )

    def package_id(self):
        self.info.header_only()