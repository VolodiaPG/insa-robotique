all : compile

compile:
	catkin_make

run: compile run_only	

run_log: compile run_only_log

run_only:
	rosrun algo algo_node 10 0.1 0.005 0.01 0.1 0 0

run_only_log:
	make run_only 2>&1 | tee log.log

.PHONY: clean

clean:
	rm -rf build/