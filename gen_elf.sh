#!/bin/bash

scons board=iot-lab_M3 toolchain=armgcc noadaptivesync=1 dagroot=1 oos_openwsn #apps=cexample
mv projects/common/03oos_openwsn_prog projects/common/03oos_openwsn_dagroot.elf

scons board=iot-lab_M3 toolchain=armgcc noadaptivesync=1 oos_openwsn #apps=cexample
mv projects/common/03oos_openwsn_prog projects/common/03oos_openwsn.elf
