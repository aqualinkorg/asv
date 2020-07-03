#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cpt.packager import ConanMultiPackager

if __name__ == "__main__":

    builder = ConanMultiPackager()
    builder.add(settings={"build_type": "Debug"}, options={}, env_vars={}, build_requires={})
    builder.add(settings={"build_type": "Release"}, options={}, env_vars={}, build_requires={})
    builder.run()