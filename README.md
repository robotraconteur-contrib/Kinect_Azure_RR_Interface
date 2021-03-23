# Kinect_Azure_RR_Interface

To build Kinect Azure interface, pull the code along with robotraconteur_companion found here: https://github.com/robotraconteur/robotraconteur_companion

In addition this package is built using Vcpkg to download other packages including robotraconteur itself 
(check RR documentation for how to install using vcpkg) as well as yaml_cpp


To build use cmake, recommended usage is with cmake gui
create build folder in files directory and specify directories for cmake build

Press configure, select Visual Studio 16 2019 for project generator

Specify platform for generator to x64

And select specify toolchain file for cross-compiling

To specify the toolchain file navigate to your vcpkg install folder and select /scripts/buildsystems/vcpkg.cmake and press Finish

If there are any errors identify which libraries are missing, if robotraconteur_companion is missing then select the entry in the cmake entry list and select it
specify the build folder in the location of your robotraconteur_companion install, so ../robotraconteur_companion/build

Configure again if necessary and then press Generate
There should now be a KinectV2.sln file in your specified build folder, open it in Visual studio, you should then be able to build and run the code from there. 
If necessary remove the ALL_BUILD directory from the Solution Explorer menu and rebuild.
