
release/built: git
	mkdir -p release
	cd release && cmake -DCMAKE_BUILD_TYPE=Release ../git
	cd release && make -j`cat /proc/cpuinfo  | grep processor | wc -l`  # Use as many cores as you have.
	touch $@

git:
	#git clone https://github.com/PointCloudLibrary/pcl.git git  # This doesn't work for me.  Why?
	git clone git@github.com:PointCloudLibrary/pcl.git git
	cd git && git checkout pcl-1.7.0

clean:
	cd release && make clean

wipe: clean
	rm -rf release git