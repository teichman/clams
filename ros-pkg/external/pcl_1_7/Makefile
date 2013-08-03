
release/built: git
	mkdir -p release
	cd release && cmake -DCMAKE_BUILD_TYPE=Release ../git
	cd release && make -j`cat /proc/cpuinfo  | grep processor | wc -l`  # Use as many cores as you have.
	touch $@

# HTTPS clone is not working on some machines.  Unclear why.  Using ssh (which requires  as a fallback.
git:
	@git clone https://github.com/PointCloudLibrary/pcl.git git; \
	RES=$$?; \
	sleep 1; \
	if [ $$RES -ne 0 ]; then \
	echo Using ssh fallback.; \
	git clone git@github.com:PointCloudLibrary/pcl.git git; \
	fi
	cd git && git checkout pcl-1.7.0

clean:
	cd release && make clean

wipe:
	rm -rf release git