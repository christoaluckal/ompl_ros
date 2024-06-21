#!c++
# https://github.com/ompl/ompl/issues/243

OMPL_DIR = /usr/local
CXX_FLAGS = -O2 
INCLUDE_FLAGS = -I/usr/include/eigen3 -I/usr/local/include/ompl-1.6
LD_FLAGS = -L${OMPL_DIR}/lib/x86_64-linux-gnu -lompl -lompl_app -lompl_app_base -l boost_system -lboost_serialization -lboost_program_options -Wl,-rpath -lompl_app_base
CXX=c++



ppm: base.o
	$(CXX) $(CXX_FLAGS) -o exe_base base.o $(LD_FLAGS) -L/usr/local/lib/
	rm base.o

	
clean:
	rm *.o
	rm exe_*
	
%.o: %.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@