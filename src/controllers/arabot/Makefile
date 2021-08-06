# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

### Generic Makefile.include for Webots controllers, physics plugins, robot
### window libraries, remote control libraries and other libraries
### to be used with GNU make
###
### Platforms: Windows, macOS, Linux
### Languages: C, C++
###
### Authors: Olivier Michel, Yvan Bourquin, Fabien Rohrer
###          Edmund Ronald, Sergei Poskriakov
###
###-----------------------------------------------------------------------------
###
### This file is meant to be included from the Makefile files located in the
### Webots projects subdirectories. It is possible to set a number of variables
### to customize the build process, i.e., add source files, compilation flags,
### include paths, libraries, etc. These variables should be set in your local
### Makefile just before including this Makefile.include. This Makefile.include
### should never be modified.
###
### Here is a description of the variables you may set in your local Makefile:
###
### ---- C Sources ----
### if your program uses several C source files:
### C_SOURCES = my_plugin.c my_clever_algo.c my_graphics.c
###
### ---- C++ Sources ----
### if your program uses several C++ source files:
### CXX_SOURCES = my_plugin.cc my_clever_algo.cpp my_graphics.c++
###
### ---- Compilation options ----
### if special compilation flags are necessary:
### CFLAGS = -Wno-unused-result
###
### ---- Linked libraries ----
### if your program needs additional libraries:
### INCLUDE = -I"/my_library_path/include"
### LIBRARIES = -L"/path/to/my/library" -lmy_library -lmy_other_library
###
### ---- Linking options ----
### if special linking flags are needed:
### LFLAGS = -s
###
### ---- Webots included libraries ----
### if you want to use the Webots C API in your C++ controller program:
### USE_C_API = true
###
### ---- Debug mode ----
### if you want to display the gcc command line for compilation and link, as
### well as the rm command details used for cleaning:
### VERBOSE = 1
###
###-----------------------------------------------------------------------------

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
#include $(WEBOTS_HOME_PATH)/resources/Makefile.include

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

ifeq ($(OSTYPE),linux)
 CFLAGS = -std=c++11
endif

PYTHON_COMMAND ?= python
ifeq (, $(shell which $(PYTHON_COMMAND) 2> /dev/null))
release debug profile clean:
	@echo "# \033[0;33m$(PYTHON_COMMAND) not installed, impossible to generate services/messages required by the ros controller, skipping ros controller\033[0m"
else
ifeq (, $(ROS_DISTRO))
release debug profile clean:
	@echo "# \033[0;33mROS not installed or 'ROS_DISTRO' not defined\033[0m"
else
ifeq (2, $(ROS_VERSION))
release debug profile clean:
	@echo "# \033[0;33mROS_DISTRO should not be a ROS2 distribution\033[0m"
else
CXX_SOURCES = $(wildcard *.cpp)
CXX_SOURCES += $(wildcard $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/Ros*.cpp)
CXX_SOURCES += $(wildcard $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/highlevel/*.cpp)

ifeq ($(OSTYPE),windows)
 arabot.exe: $(CXX_SOURCES:.cxx=.d)
else
 arabot: $(CXX_SOURCES:.cxx=.d)
endif

INCLUDE = -I$(WEBOTS_HOME_PATH)/projects/default/controllers/ros -isystem $(WEBOTS_HOME_PATH)/projects/default/controllers/ros/include -isystem /opt/ros/$(ROS_DISTRO)/include
LIBRARIES += -lCppDriver -lCppCar -ldriver -lcar

# include ros libraries

LIBRARIES += -L/opt/ros/$(ROS_DISTRO)/lib -Wl,-rpath-link=/opt/ros/$(ROS_DISTRO)/lib -lxmlrpcpp -lcpp_common -lrosconsole_backend_interface -lroscpp -lrosconsole -lrosconsole_log4cxx -lroscpp_serialization -lrostime -lcontroller_manager -lboost_system
ifeq ($(OSTYPE),windows)
 LIBRARIES += -lws2_32
 ifeq ($(MAKECMDGOALS),debug)
  # The following option is fixing the following error
  # appearing only on Windows 64 bits in debug mode
  # (probably that the number of templates is causing this)
  # RosSupervisor.o: too many sections
  CFLAGS += -Wa,-mbig-obj
 endif
endif

ifeq ($(OSTYPE),linux)
 LIBRARIES += -Wl,-rpath,$(WEBOTS_HOME_PATH)/projects/default/controllers/ros/lib/ros
endif

ifeq ($(OSTYPE),darwin)
# Hide Boost warnings
CFLAGS += -Wno-unused-local-typedefs
# Fix warnings about the 'override' keyword.
CFLAGS += -std=c++11
endif

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
endif
endif
endif
