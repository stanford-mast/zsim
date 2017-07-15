# Common deps
DEPS=Makefile

.PHONY: default
default: test-timers

test-timers: $(DEPS) test-timers.cpp
	g++ -Wall -Wextra -pedantic -std=c++11 -pthread -g test-timers.cpp -o $@

.PHONY: run-tests
run-tests: default
	@./test-timers

.PHONY: clean
clean:
	@rm -f test-timers
