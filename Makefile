#camtest: camtest.cpp
#	g++ -g -o $@ $^ `pkg-config --cflags --libs opencv`

run_newauto: run_newauto.cpp
	g++ -g -o $@ $^ `pkg-config --cflags --libs opencv` -I /usr/local/Aria/include/ -L /usr/local/Aria/lib -lAria

run_auto: run_auto.cpp
	g++ -g -o $@ $^ `pkg-config --cflags --libs opencv` -I /usr/local/Aria/include/ -L /usr/local/Aria/lib -lAria

run: run.cpp
	g++ -g -o $@ $^ `pkg-config --cflags --libs opencv` -I /usr/local/Aria/include/ -L /usr/local/Aria/lib -lAria
