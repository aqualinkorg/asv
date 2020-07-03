from conans import ConanFile, CMake, tools
import os


class ConcurrentQueueConan(ConanFile):
    name = "concurrentqueue"
    version = "1.0.0"
    description = "A single-producer, single-consumer lock-free queue for C++"
    license = ("Simplified BSD", "zlib")
    repo_url = "https://github.com/cameron314/concurrentqueue"
    author = "Aqualink Developer <developer@aqualink.org>"

    sha1 = "dea078cf5b6e742cd67a0d725e36f872feca4de4"

    def source(self):
        tools.get("%s/archive/%s.zip" % (self.repo_url, self.sha1))

    def package(self):
        self.copy( "LICENSE.md",                src="%s-%s" % ( self.name, self.sha1 ) )
        self.copy( "concurrentqueue.h",         src="%s-%s" % ( self.name, self.sha1 ) , dst="include/%s" % self.name )
        self.copy( "blockingconcurrentqueue.h", src="%s-%s" % ( self.name, self.sha1 ) , dst="include/%s" % self.name )

    def package_info(self):
        self.cpp_info.libs = ["rt", "pthread"]

    def package_id(self):
        self.info.header_only()
    