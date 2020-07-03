#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cpt.packager import ConanMultiPackager

if __name__ == "__main__":

    builder = builder = ConanMultiPackager()
    builder.add()
    builder.run()