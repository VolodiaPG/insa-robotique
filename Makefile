all : compile

compile:
	catkin_make

run: compile run_only	

run_log: compile run_only_log

run_only:
	rosrun algo algo_node -1  \
		0.4 0.00 0.02 \
		0.7 0.0 0.0 \
		0.15 0 0 \
		-0.018 0 0 \
		0.5 0.3
	# rosrun algo algo_node 1 0.02 0.1 0.0001 0.1 1 0.002 1 0 0 1.5 0.3

run_only_log:
	make run_only > log.log

.PHONY: clean

clean:
	rm -rf build/