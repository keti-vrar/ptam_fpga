all:
	cd ptam_com && make -j8 -g
	cd ..
	cd ptam && make -j8 -g
	cd ..

clean: 
	cd ptam && make clean 
	cd ..
	cd ptam_com && make clean
	cd ..
	
	
distclean: 
	cd ptam && make distclean 
	cd ..
	cd ptam_com && make distclean
	cd ..
