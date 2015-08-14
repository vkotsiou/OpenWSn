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

submitsmall:
	../cli-tools-1.6.0/experiment-cli submit -n trackBalancing -d 7 \
		-l grenoble,m3,33+35+37+40+42+45+48+50+55,projects/common/03oos_openwsn.elf \
		-l grenoble,m3,60,projects/common/03oos_openwsn_dagroot.elf 
submit:
	../cli-tools-1.6.0/experiment-cli submit -n trackBalancing -d 7 \
		-l grenoble,m3,33-35+37-40+42-45+48-50+55,projects/common/03oos_openwsn.elf \
		-l grenoble,m3,60,projects/common/03oos_openwsn_dagroot.elf 

submitapp:
	../cli-tools-1.6.0/experiment-cli submit -n trackBalancing -d 10 \
		-l grenoble,m3,33,projects/common/03oos_openwsn_app.elf \
		-l grenoble,m3,34-35+37-40+42-45+48-50+55,projects/common/03oos_openwsn.elf \
		-l grenoble,m3,60,projects/common/03oos_openwsn_dagroot.elf 
submitappsmall:
	../cli-tools-1.6.0/experiment-cli submit -n trackBalancing -d 10 \
		-l grenoble,m3,33,projects/common/03oos_openwsn_app.elf \
		-l grenoble,m3,42+45+48+50+55,projects/common/03oos_openwsn.elf \
		-l grenoble,m3,60,projects/common/03oos_openwsn_dagroot.elf 
rsync:
	rsync -av --delete-after --exclude '.sconsign.dblite' --exclude 'build' --exclude 'projects/common'  ../exp-iotlab/openwsn/openwsn-fw/ ../exp-iotlab/openwsn/openwsn-fw-sink/

clean:
	find * -name '*.o' | xargs rm -rf
	rm -r build/iot*
