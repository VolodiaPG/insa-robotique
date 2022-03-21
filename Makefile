all : compile

compile:
	catkin_make

run: compile run_only	

run_log: compile run_only_log

run_only:
	rosrun algo algo_node 15 0.02 0.1 0.0001 0.1 1 0.002 1 0 0

run_only_log:
	make run_only 2>&1 | tee log.log

.PHONY: clean

clean:
	rm -rf build/