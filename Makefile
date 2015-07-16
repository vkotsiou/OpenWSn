all:
	scons board=iot-lab_M3 toolchain=armgcc noadaptivesync=1 dagroot=1 oos_openwsn
	mv projects/common/03oos_openwsn_prog projects/common/03oos_openwsn_dagroot.elf
	scons board=iot-lab_M3 toolchain=armgcc noadaptivesync=1 oos_openwsn 
	mv projects/common/03oos_openwsn_prog projects/common/03oos_openwsn.elf
app:
	scons board=iot-lab_M3 toolchain=armgcc noadaptivesync=1 dagroot=1 oos_openwsn
	mv projects/common/03oos_openwsn_prog projects/common/03oos_openwsn_dagroot.elf
	scons board=iot-lab_M3 toolchain=armgcc noadaptivesync=1 oos_openwsn 
	mv projects/common/03oos_openwsn_prog projects/common/03oos_openwsn.elf
	scons board=iot-lab_M3 toolchain=armgcc noadaptivesync=1 oos_openwsn apps=cexample
	mv projects/common/03oos_openwsn_prog projects/common/03oos_openwsn_app.elf

submit:
	../cli-tools-1.6.0/experiment-cli submit -n trackBalancing -d 7 \
		-l grenoble,m3,40+44+45+50,projects/common/03oos_openwsn.elf \
		-l grenoble,m3,60,projects/common/03oos_openwsn_dagroot.elf 

submitapp:
	../cli-tools-1.6.0/experiment-cli submit -n trackBalancing -d 7 \
		-l grenoble,m3,40,projects/common/03oos_openwsn_app.elf \
		-l grenoble,m3,44+45+50,projects/common/03oos_openwsn.elf \
		-l grenoble,m3,60,projects/common/03oos_openwsn_dagroot.elf 
rsync:
	 rsync -av --delete-after --exclude '.sconsign.dblite' --exclude 'build' --exclude 'projects/common'  ../exp-iotlab/openwsn/openwsn-fw/ ../exp-iotlab/openwsn/openwsn-fw-sink/

