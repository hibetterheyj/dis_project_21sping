# dis_project_21spring

> [[Google Doc](https://docs.google.com/document/d/16DUTJ36V0xkJ0E7SUYr_FLohmIjJYjbZGZQweGHfsQA/edit?usp=sharing)]

## Teamwork

- Localization: Nicola
- High-level control
  - Flocking: Qi
  - Formation: Yujie
  - PSO: Jianhao
- Misc.
  - Low-level control
  - Analysis and visualization

## Schedule:

- Localization: 03.05 - 09.05
- Flocking & Formation: 10.05 - 16.05
- PSO: 17.05 - 23.05
- Unexpected issues: 24.05 - 30.05
- Report: 31.05 - 04.06

## Links

- GSL for matrix operation: https://www.gnu.org/software/gsl/doc/html/intro.html

- A cheat sheet for Git workflows: https://github.com/hbons/git-cheat-sheet

  ![](https://raw.githubusercontent.com/hbons/git-cheat-sheet/master/git-cheat-sheet.svg)

## use gsl locally on Ubuntu

- install

```shell
#Download gsl-latest.tar.gz from the GSL ftp site and unzip it anywhere (e.g. /Downloads)
wget https://mirror.easyname.at/gnu/gsl/gsl-2.7.tar.gz
#Open the unzipped gsl folder in Terminal (e.g. cd ~/Downloads/gsl-2.4.
tar -zxvf gsl-2.7.tar.gz
#Run sudo ./configure && make && make install.
mkdir /home/yhe/gsl
./configure --prefix=/home/yhe/gsl
make
make install
```

- environment setup

```shell
vi ~/.bashrc
# add following lines
export LD_LIBRARY_PATH==$LD_LIBRARY_PATH:/home/yhe/gsl/lib
export CFLAGS="-I/home/yhe/gsl/include"
export LDFLAGS="-L/home/yhe/gsl/lib"
source ~/.bashrc
```

- changes in MakeFile

```makefile
### Do not modify: this includes Webots global Makefile.include
C_SOURCES = formation_graph_crossing_controller.c ../localization_library_test/localization.c ../localization_library_test/trajectories.c ../localization_library_test/odometry.c ../localization_library_test/kalman.c

# yujie
# https://stackoverflow.com/questions/714100/os-detecting-makefile
UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
	#LIBRARIES = -lm -lgsl -lgslcblas
	INCLUDE = -I"/home/yhe/gsl/include"
	LIBRARIES = -L"/home/yhe/gsl/lib" -lm -lgsl -lgslcblas
else
	INCLUDE = -I"C:\msys64\mingw64\include"
	LIBRARIES = -L"C:\msys64\mingw64\lib" -llibgsl
endif

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
```

- if error still happens, copy the erroring library: `error while loading shared libraries: libgsl.so.25: cannot open shared object file: No such file or di`

  ```shell
  cp libgsl.so.25 ~/Desktop/dis_project_21sping/Initial_Material/controllers/formation_graph_crossing_controller/
  ```

  - ref: https://stackoverflow.com/questions/22222666/error-while-loading-shared-libraries-libgsl-so-0-cannot-open-shared-object-fil/37407181
