CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2
SOURCES = main.cpp catalog.cpp engine.cpp paschen.cpp ignition.cpp cranking.cpp records.cpp sparks.cpp diagnosis.cpp
HEADERS = catalog.hpp engine.hpp paschen.hpp ignition.hpp cranking.hpp records.hpp sparks.hpp diagnosis.hpp linalg.hpp fit.hpp
TEST_SOURCES = host_test.cpp catalog.cpp engine.cpp paschen.cpp ignition.cpp cranking.cpp records.cpp sparks.cpp diagnosis.cpp

spark_clip: $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES)

host_test: $(TEST_SOURCES) $(HEADERS)
	$(CXX) -std=c++17 -Wall -Wextra -O1 -g -fsanitize=address,undefined -o $@ $(TEST_SOURCES)

check: host_test
	./host_test ..

clean:
	rm -f spark_clip host_test