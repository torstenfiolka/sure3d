----------------
SURE 3D Features
----------------

Version: 0.1, October 2012

This directory contains the source code for the SURE 3D Features.
Please note that the code API is _NOT_ stable and might change in future versions.  

If you use the code, please cite:
	SURE: Surface Entropy for Distinctive 3D Features
	Torsten Fiolka, Jörg Stückler, Dominik A. Klein, Dirk Schulz, Sven Behnke
	Spatial Cognition 2012

bibtex:
@incollection {
   author = {Fiolka, Torsten and Stückler, Jörg and Klein, Dominik and Schulz, Dirk and Behnke, Sven},
   title = {SURE: Surface Entropy for Distinctive 3D Features},
   booktitle = {Spatial Cognition VIII},
   series = {Lecture Notes in Computer Science},
   publisher = {Springer Berlin / Heidelberg},
   pages = {74-93},
   year = {2012}
}

Further, parts of the octreelib (http://ros.org/wiki/octreelib) from Jörg Stückler are included.

------------
INSTALLATION
------------

The project was developed under Ubuntu Linux 12.04 and not tested in other operating systems.

Dependencies:
	-- CMAKE 2.6 or later
	-- PCL 1.5.1 (www.pointclouds.org)
	-- FastEMD from Ofir Pele and Michael Werman (NOT INCLUDED!)
	   Tested with versions 2.1 to 3.0 from http://www.seas.upenn.edu/~ofirpele/FastEMD/code/
	   You need to place the FastEMD directory somewhere the compiler can find it, default is
	   include/FastEMD. You can change the include-path in include/sure/histogram_emd.h, if necessary.
	-- an early version of the octreelib from Jörg Stückler (included)
	
To compile the code, switch to the directory and type:
cmake .
make

To use it, include the needed header files into your project and make sure that the linker
can find the library.

-----
USAGE
-----

To uses SURE 3D Features you need to include sure/sure_estimator.h in your own code.
Create an object of SURE_Estimator<pcl::PointType> with pcl::PointType corresponding
to the type of point cloud you use, e.g. pcl::PointXYZRGB.

To change parameters, create an object of sure::Configuration, alter the parameters and call
SURE_Estimator::setConfig(sure::Configuration& altered_configuration) to use them.

The use setInpudCloud(pcl::PointCloud<..>&) to insert the point cloud and call 
SURE_Estimator::calculateFeatures() to start the feature detection and extraction.

To access the calculated features, use getFeatures() or getInterestPoints().

A little example of the usage combined with some visualization is include in src/test_cure.cpp
After compiling, you should be able to start the program bin/test_sure.    

-------
CONTACT
-------

If you have questions, comments or found bugs feel free to contact me under
torstenfiolka@googlemail.com

You can also use the bug tracker on the repository site:
http://code.google.com/p/sure-3d-features/
  


   