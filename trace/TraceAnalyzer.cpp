//============================================================================
// Name        : TraceAnalyzer.cpp
// Author      : Elmano R. Cavalcanti
// Version     : 2.1 (May 13 2011)
// Copyright   :
// Description : Mobility Trace Analyzer (NS-2 compatible)
//============================================================================
/*
* TraceAnalyzer.cpp
* License: GPL 2.0
*  Version 2.1: May 13 2011
*      Author: elmano
*      elmano.cavalcanti@garanhuns.ifpe.edu.br
*  Modified: 2017/01-05
*      Last modified: Matheus Henrique Trichez - UFFS
*      email: mh.trichez@gmail.com
*/
#include <ctime>
#include <string>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "LinkAnalysis.h"

using namespace std;
#define length(a) ( sizeof ( a ) / sizeof ( *a ) )

int main(int argc, char* argv[]){

	int i;
	int R;
	double width;
	double length;
	int rounds;
	int n_batch;
	int node_num;  //100
	int time_slot; //900
	int node_pause_time;
	int node_pause_time_slot; //node_pause_time_slot
	int	max_number_waypoints; //50 waypoints = visiting points
	int node_trip_length_slot;   //50
	char * current_path = new char[4096];
	char * nameModelParam = NULL;
	char * nameModelAD = NULL;

	ifstream batch_file;
	ofstream match;

	std::string run;
	std::string params;
	std::string file_name;
	tree *t;
	Trace * mytrace;

	t = decisionTreeInicializer(t);
	srand(time(NULL));
	getcwd(current_path, 4096);
	std::string bm_run(current_path);
	delete [] current_path;

	bm_run += "/bonnmotion-3.0.1/bin/bm";									// that just doesn't smells good

	if(strcmp(argv[1], "-f") == 0){

		for(i = 2; strcmp(argv[i], "-trace") != 0 ; i++){
			params.append(" ");
			params.append(argv[i]);
		}
		run = bm_run + params;
		nameModelParam = argv[2];
		//--------------
		rounds =	atoi(argv[++i]);
		R = atoi(argv[++i]);
		node_pause_time = atoi(argv[++i]);
		width = atof(argv[++i]);
		length = atof(argv[++i]);
		node_num = atoi(argv[++i]);
		time_slot = atoi(argv[++i]);

		node_pause_time_slot = 50; //
		node_trip_length_slot = 50;
		max_number_waypoints = 50;//
	//-------------

		for(i = 0; i < rounds;  i++){ // laço que executa rounds vezes os testes

			file_name = to_string(clock()) + genRandomS(5) + formatParams(params.c_str()) ;			 // random name
			run = bm_run + " -f " + file_name + params + " > garbage.txt";

			if(system(run.c_str()) == -1){
				std::cout << "couldn't execute BM" << '\n';
				exit(0);
			}

			run = bm_run + " NSFile -b -f " + file_name + " > garbage.txt";
			if(system(run.c_str()) == -1){
				std::cout << "couldn't execute BM conversion to NS2" << '\n';
				exit(0);
			}
			file_name += ".ns_movements";
			std::cout << "analyzing file: " << file_name << '\n';
			std::cout << "allocating memory..." << '\n';

			mytrace = (Trace *) malloc(sizeof(Trace));
			new (mytrace) Trace(node_num, time_slot, node_pause_time, node_pause_time_slot, max_number_waypoints, node_trip_length_slot, width, length, R);

			if(i == 0 )
				trace = twoD_array_allocData(node_num, time_slot);// a instanciação de trace precisa vir antes de mytrace (sujeito a seg. fault)

			std::cout << "reading data..." << '\n';
			mytrace->initiate();
			mytrace->read_trace(file_name.c_str());
			mytrace->set_data();

			std::cout << "calculating metrics..." << '\n';
			char link_distribution[] = "link_distribution.txt";
			mytrace->cal_link(link_distribution);
			char node_degree_dist[] = "node_degree_dist.txt";
			mytrace->cal_node_degree(node_degree_dist);
			mytrace->average_relative_speed();
			mytrace->degree_of_spatial_dependence();
			mytrace->average_coverage();
			mytrace->calculateMetrics();

			nameModelAD = modelIdentifier(t->root, mytrace->metrics);
			mytrace->writeResults("match.txt");
			match.open("match.txt", std::ofstream::app);

			if(!match.is_open()){
				std::cout << "couldn't open match file" << '\n';
				exit(0);
			}else
				if(strcmp(nameModelParam, nameModelAD) == 0)
					match << " 1 | "  << file_name << '\n' << "------------------------------------------" << '\n';
				else
					match << " 0 | " << file_name << " -> " << nameModelAD << '\n';

			match.close();
			mytrace->printMetrics();
			mytrace->release_mem();
			trace = reset_trace(trace, node_num, time_slot);
		}
	}else if(strcmp(argv[1], "-bf") == 0){
		int itrace;
		for (itrace = 0; strcmp(argv[itrace], "-trace") != 0 ; itrace++){} //searches for the flag '-trace'

		++itrace;
		R = atoi(argv[++itrace]);
		node_pause_time = atoi(argv[++itrace]);
		width = atof(argv[++itrace]);
		length = atof(argv[++itrace]);
		node_num = atoi(argv[++itrace]);
		time_slot = atoi(argv[++itrace]);
		node_pause_time_slot = 50;
		max_number_waypoints = 50;
		node_trip_length_slot = max_number_waypoints;
		params = argv[2];

		if(params.find(".txt") == string::npos){
			std::cout << " .txt file not found!" << '\n';
			exit(0);
		}else{
			batch_file.open(params.c_str());
			if(!batch_file.is_open()){
				std::cout << " .txt file couldn't be opened!" << '\n';
				exit(0);
			}

			batch_file >> rounds;

			for(i = 0; i < rounds; i++) {
				batch_file >> file_name;
				std::cout << "analyzing file: " << file_name << '\n';
				std::cout << "allocating memory..." << '\n';

				mytrace = (Trace *) malloc(sizeof(Trace));
				new (mytrace) Trace(node_num, time_slot, node_pause_time, node_pause_time_slot, max_number_waypoints, node_trip_length_slot, width, length, R);
				if(i == 0 )
					trace = twoD_array_allocData(node_num, time_slot);// a instanciação de trace precisa vir antes de mytrace (sujeito a seg. fault)

				std::cout << "reading data..." << '\n';
				mytrace->initiate();
				mytrace->read_trace(file_name.c_str());
		  		mytrace->set_data();

				char link_distribution[] = "link_distribution.txt";
				mytrace->cal_link(link_distribution);
				char node_degree_dist[] = "node_degree_dist.txt";
				mytrace->cal_node_degree(node_degree_dist);
				mytrace->average_relative_speed(); //Relative Speed
				mytrace->degree_of_spatial_dependence(); //Degree of Spatial Dependence
				mytrace->average_coverage(); //Average Coverage

				mytrace->calculateMetrics();
				nameModelAD = modelIdentifier(t->root, mytrace->metrics);

				mytrace->writeResults("match.txt");
				match.open("match.txt", std::ofstream::app);
				if(!match.is_open()){
					std::cout << "couldn't open match file" << '\n';
					exit(0);
				}else
					match << file_name << " is: " << nameModelAD << '\n' << "------------------------------------------" << '\n';
				match.close();

				mytrace->release_mem();
				trace = reset_trace(trace, node_num, time_slot);
				//---------------------------------------
			}
		}
	}else{ // refuses the entry
		std::cout << "invalid entry! " << '\n' << "tip: README " << '\n';
		exit(0);
	}
	return 0;
}
