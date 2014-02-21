Function blocks for the BRICS_3D library
==========================================================


Dependencies
------------

 - BRICS_3D library. Installation instructions can be found here: http://www.best-of-robotics.org/brics_3d/installation.html
 - microblx library. See: https://github.com/kmarkus/microblx

Here is a summuary on how to install microblx on Ubuntu (12.04)

Dependencies:
```
	sudo apt-get install clang

	wget luajit.org/download/LuaJIT-2.0.2.tar.gz
	tar -xvf LuaJIT-2.0.2.tar.gz 
	cd LuaJIT-2.0.2
	make
	sudo make install
	sudo ln -s /usr/local/bin/luajit /usr/local/bin/lua
	sudo ldconfig
```

Installation of the actual microblx library:
```
	git clone https://github.com/kmarkus/microblx.git
	git checkout dev # you might want to skip this step
	cd microblx
	source env.sh 
	make
	echo "export MICROBLX_DIR=$PWD" >> ~/.bashrc
```

Compilation
-----------

```
 $ mkdir build
 $ cd build 
 $ cmake .. -DCMAKE_CXX_COMPILER=/usr/bin/clang++
 $ make 
```

Usage
-----

TBD

Licensing
---------

This software is published under a dual-license: GNU Lesser General Public
License LGPL 2.1 and Modified BSD license. The dual-license implies that
users of this code may choose which terms they prefer. Please see the files
called LGPL-2.1 and BSDlicense.


Impressum
---------

Written by Sebastian Blumenthal (blumenthal@locomotec.com)
Last update: 24.01.2014
 


