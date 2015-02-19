#!/bin/bash


sudo scons board=telosb toolchain=mspgcc apps=cexample bootload=/dev/ttyUSB$1 oos_openwsn

