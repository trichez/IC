/*
 * LinkAnalysis.h
 *
 *  Created on: 04/02/2010
 *      Author: elmano
 *      elmano.cavalcanti@garanhuns.ifpe.edu.br
 *  Last modified: 2017/01-05
 *      Modifier: Matheus Henrique Trichez - UFFS
 *      email: mh.trichez@gmail.com
 */
#include "misc.h"
#include <iostream>
#include <fstream>
#define NIL -999 //constant


//-----------------------------------------------------------------//

struct data{
	int id;
	double time, start;
	double x, y;
	double next_x, next_y;
	double angle, speed;
	int timetodestiny;
};

struct point {
	double x, y;
	double time;
};
//raw data
struct data** trace;

struct point** waypoints;

class Trace{

	public:
		int time_slot;
		int node_num;
		int max_number_Waypoints;  														//waypoints = visiting points
		int node_pause_time_slot;  														//array that keeps all pause time durations of a node // vetor que guarda todas as durações dos tempos de pausa
		int node_trip_length_slot;

		int nodes_pause_time_slot;  													//node_num*node_pause_time_slot 		| array that keeps all pause time durations of all nodes
		int node_path_length_slot;  													//node_trip_length_slot   				| same size
		int nodes_trip_length_slot; 													//node_num * node_trip_length_slot
		int nodes_path_length_slot; 													//nodes_trip_length_slot 			   	| same size
		int node_speed_slot;       													  	//time_slot 							| the speed values of a node (/10 otherwise the array would be to large)
		int nodes_speed_slot;    														//node_num * node_speed_slot 	  		|
		int node_angle_slot;        													//time_slot								| the velocity angle values of a node (upper bound = 1, i.e. angle changes at every time stamp)
		int nodes_angle_slot;       													//node_num * node_angle_slot			|

	private:
		//used for link calculation
		int ** link_status;																//[node_num][node_num];
		int ** link_up_num;																//[node_num][node_num];
		int ** link_down_num;															//[node_num][node_num];
		int ** last_up_time;															//[node_num][node_num];
		int ** last_down_time;															//[node_num][node_num];


	public:
		double *** DSD;																	//[node_num][node_num][time_slot]
		double *** estimatedIDSD; 														//[node_num][node_num][time_slot] 		| to keep all the historic of all spatial correlations, by Elmano
		double *** estimatedHIDSD;														//[node_num][node_num][time_slot]
		double ** temporalCorrelations; 												//[node_num][time_slot]           		| to keep all the historic of all temporal correlations, by Elmano
		double ***distances; 															//[node_num][node_num][time_slot] 		| to keep all the distances between nodes at all time steps, by Elmano
		double ** average_distances;          											//[node_num][node_num]					| to keep the average of the distances between nodes, by Elmano
		double ** std_distances; 														//[node_num][node_num]					| to keep the std of the distances between nodes, by Elmano

		double ** nodePathLengths; 														//[node_num][node_pause_time_slot]		| to keep the trip DISTANCE of all nodes
		double * nodesPathLengths; 														//[nodes_trip_length_slot]
		double ** nodeTripLengths; 														//[node_num][node_pause_time_slot]		| to keep the trip DISPLACEMENT of all nodes
		double * nodesTripLengths;														//[nodes_trip_length_slot]
		double ** nodePauseTimes; 														//[node_num][node_pause_time_slot]		| to keep the values of pause duration of all nodes (i.e., the duration of time where the node was stationary)
		double * nodesPauseTimes; 														//[nodes_pause_time_slot]
		double ** nodeSpeeds; 															//[node_num][node_speed_slot] 			| the values of speed of each node (in the upper bound each node has time_slot values, i.e., one
		double * nodesSpeeds; 															//[nodes_speed_slot] 					| the values of speed of all nodes
		double ** nodeAngles; 															//[node_num][node_angle_slot] 			| the values of velocity angle of each node (in the upper bound each node has time_slot values)
		double * nodesAngles;															//[nodes_angle_slot						| the values of velocity angle of all nodes
		double degreeSpatialDistribution;

		int **cells; 																	//[node_num+1][node_num+1] 				| store the total amount of nodes in each cell during the simulation

		double SCENARIO_WIDTH;
		double SCENARIO_LENGTH;
		int RADIUS;
		int TIMEPAUSE; //AVERAGE TIME PAUSE

		int GRID_WIDTH;
		int GRID_LENGTH;
		double CELL_WIDTH;
		double CELL_LENGTH;

		int total_link_change;															//		                                | Número Total de Mudanças de Enlaces (TME) - calculada em cal_link
		double avg_link_duration;													    //                                      | Duração de Enlace Relativa (DER)- calculada em cal_link
		int *ld_pdf;

		double node_degree;
		double *nd_pdf;
		double static_degree;
		int *static_pdf;

		double cluster_coefficient;

		double * ini_x;
		double * ini_y;
		//---------------------new methods, by Trichez----------
		void verificaTrace(data **t);
		double * metrics;																															// Keeps the metrics calculated values for the comparison on the decision tree
		Trace();
		Trace(int nodeNUM, int time_slot, int timePause, int node_pause_time_slot, int max_number_Waypoints, int node_trip_length_slot, double width, double lenght, int R);
		int ** twoD_array_allocI(int x, int y);
		int ** release_array_twoD_arrayI(int **m, int cols);
		double ** twoD_array_allocF(int x, int y);
		double *** threeD_array_allocF(int x, int y, int z);
		double ** release_array_twoD_arrayF(double **m, int cols);
		double *** release_array_threeD_arrayF(double ***m, int rows, int cols);
		data ** release_array_twoD_arrayD(data **m, int cols);
		double degreeOfLinkChanges();													//DLC -
		double degreeOfSpatialAcessibility();											//DSA - must come after DNSD
		void release_mem();
		void printTrace(data **m, int i, int j);
		void calculateMetrics();
		void printMetrics();
		void writeResults(const char *file_name);
// -----------------------------------------
		double mean(int* pdf);
// ------------ adopted methods /\ ---------
		void initiate();
		void setAngleSpeedArrays();
		void show_trace();
		void show_trace(int start, int end);
		void show_trace(int node, int start, int end);
		void read_trace(const char *filename);
		void set_data();

		void cal_link(char* filename);
		void cal_node_degree(char* filename);
		void cal_static_degree(char* filename);

		//new functions (version 2.0) by Elmano
		double degree_of_spatial_dependence(); //DSD
		double degree_of_temporal_dependence(); //DTD
		double improved_degree_of_spatial_dependence(); //IDSD
		double high_improved_degree_of_spatial_dependence(); //HIDSD
		double improved_degree_of_temporal_dependence(); //IDTD
		double average_relative_speed();
		//void is_there_correlation_between_nodes_i_j(int i, int j);
		void print_spatial_dependence_statistics(int i, int j);
		void print_DSD(int i, int j, int start, int end);
		void print_IDSD(int i, int j, int start, int end);
		void print_HIDSD(int i, int j, int start, int end);
		void print_SpatialMetrics(int i, int j, int start, int end);
		bool is_stopped(int i, int t);
		bool stop_trip(int i, int t);
		bool velocity_has_changed(int i, int t);
		bool equal_or_almost_equal(double x, double y);
		void average_distance_and_coverage();
		double average_distance();
		double average_distance(int x, int y);
		double average_coverage();																										//Probabilidade de Cobertura Nodal (PCN)
		double average_coverage_i_j(int x, int y);
		double distance_i_j(int i, int j, int t);
		double SMAverage(int i, int j, int t, int periods);
		double WMAverage(int i, int j, int t, int periods);
		double EWMAverage(int i, int j, int t);
		double compute_mean_basic_temporal_correlation(int i, int t);
		void test_is_stopped(int x, int y);
		void test_velocity_has_changed(int x, int y);
		void print_distance_between_nodes();
		void average_std_distances();
		double degree_of_node_proximity();																						//Grau de Proximidade entre Nós (GPN)
		double improved_degree_of_node_proximity();
		double maximum_average_distance();
		double average_tripLength();																									//Deslocamento Médio de Percurso (DeP)
		double average_pathLength();																									//Distância Média de Percurso (DiP)
		bool node_is_stationary(int id, int t);
		bool is_at_previous_position(int id, int t);
		bool is_at_destination(int id, int t);
		bool no_next_xy(int id, int t);
		bool same_destination(int id, int t);
		void update_stationary_state(int id, int t);
		int update_moving_state(int id, int t);
		int start_new_movement(int id, int t);
		void printSingleTrace(int i, int t);
		void printPauseTimes(int i);
		void printPauseTimes();
		void printAngles(int i);
		void printAngles();
		void printSpeeds(int i);
		void printSpeeds();
		void printTripLengths(int i);
		void printTripLengths();
		void printPathLengths(int i);
		void printPathLengths();
		void fillNodesPathLengths();
		void fillNodesPauseTimes();
		void fillNodesSpeeds();
		void fillNodesAngles();
		void fillDistances();
		void fillNodesTripLengths();
		double getPositiveAngle(double a);
		double speed_angle_rate();
		double speed_angle_rate(int i);
		double angle_variation_coefficient();																				//Coeficiente de Variação de Ângulo(CVA)
		double speed_variation_coefficient();																				//Coeficiente de Variação de Velocidade(CVV)
		double pause_variation_coefficient();																				//Coeficiente de Variação de Tempo de Parada(CVP)
		double degreeOfSpatialDistribution();																				//Grau de Distribuição Espacial da Rede (GDER)
		double positionDensityVariance();																						//Variação da Densidade de Posição
		double emptyCells();																													//REC - must come after DNSD
		int getLength(double array[], int max);
		void createIdentityVector(int size);
		void xy_axis_interval();
		double distanceTraveled(int id, int t1, int t2);

};

/* --------- Membership Function and constructors ------------*/
Trace::Trace(){
	printf("empty constructor\n");
}

Trace::Trace(int nodeNUM, int time_slot, int timePause, int node_pause_time_slot, int max_number_Waypoints, int node_trip_length_slot, double width, double lenght, int R){

	this->RADIUS = R;
	this->TIMEPAUSE = timePause; //node_pause_time_slot
	this->SCENARIO_WIDTH = width;
	this->SCENARIO_LENGTH = lenght;
	this->time_slot = time_slot; //900
	this->node_num = nodeNUM;  //100
	this->max_number_Waypoints = max_number_Waypoints; //50 waypoints = visiting points
	this->node_pause_time_slot  = node_pause_time_slot;// 60 array that keeps all pause time durations of a node
	this->node_trip_length_slot = node_trip_length_slot;   //50

	this->nodes_pause_time_slot = this->node_num * this->node_pause_time_slot; //array that keeps all pause time durations of all nodes
	this->node_path_length_slot = this->node_trip_length_slot; //same size
	this->nodes_trip_length_slot = this->node_num * this->node_trip_length_slot;
	this->nodes_path_length_slot = this->nodes_trip_length_slot; //same size
	this->node_speed_slot = this->time_slot; //the speed values of a node (/10 otherwise the array would be to large)
	this->nodes_speed_slot = this->node_num * this->node_speed_slot;
	this->node_angle_slot = this->time_slot; //the velocity angle values of a node (upper bound = 1, i.e. angle changes at every time stamp)
	this->nodes_angle_slot = this->node_num * this->node_angle_slot;


	link_status = twoD_array_allocI(node_num, node_num);
	link_up_num = twoD_array_allocI(node_num, node_num);
	link_down_num = twoD_array_allocI(node_num, node_num);
	last_up_time = twoD_array_allocI(node_num, node_num);
	last_down_time = twoD_array_allocI(node_num, node_num);

	DSD = threeD_array_allocF(node_num, node_num, time_slot);
	estimatedIDSD = threeD_array_allocF(node_num, node_num, time_slot);
	estimatedHIDSD = threeD_array_allocF(node_num, node_num, time_slot);
	temporalCorrelations = twoD_array_allocF(node_num, time_slot);
	distances = threeD_array_allocF(node_num, node_num, time_slot);
	average_distances = twoD_array_allocF(node_num, node_num);
	std_distances = twoD_array_allocF(node_num, node_num);

	nodePathLengths = twoD_array_allocF(node_num, node_pause_time_slot);
	nodesPathLengths  = (double *) malloc(nodes_trip_length_slot * sizeof(double));
	nodeTripLengths = twoD_array_allocF(node_num, node_pause_time_slot);
	nodesTripLengths =(double *) malloc(nodes_trip_length_slot * sizeof(double));
	nodePauseTimes = twoD_array_allocF(node_num, node_pause_time_slot);
	nodesPauseTimes = (double *) malloc(nodes_trip_length_slot * sizeof(double));
	nodeSpeeds = twoD_array_allocF(node_num, node_speed_slot);
	nodesSpeeds = (double *) malloc(nodes_speed_slot * sizeof(double));
	nodeAngles = twoD_array_allocF(node_num, node_angle_slot);
	nodesAngles = (double *) malloc(nodes_angle_slot * sizeof(double));

	cells = twoD_array_allocI(node_num +1,node_num +1);
	ld_pdf = (int *) malloc((time_slot + 1) * sizeof(int));
	nd_pdf = (double *) malloc((time_slot + 1) * sizeof(double));
	static_pdf = (int *) malloc((time_slot + 1) * sizeof(int));

	ini_x	= (double *) malloc(node_num * sizeof(double));
	ini_y = (double *) malloc(node_num * sizeof(double));

	metrics = (double *) malloc(11 * sizeof(double));		/*ler esse 11 do treeconf.dat number of nodes on the tree*/														

}

void Trace::release_mem(){

	if(this->link_status )
		this->link_status = release_array_twoD_arrayI(this->link_status, this->node_num);
	if(this->link_up_num )
		this->link_up_num = release_array_twoD_arrayI(this->link_up_num, this->node_num);
	if(this->link_down_num )
		this->link_down_num = release_array_twoD_arrayI(this->link_down_num, this->node_num);
	if(this->last_up_time )
		this->last_up_time = release_array_twoD_arrayI(this->last_up_time, this->node_num);
	if(this->last_down_time )
		this->last_down_time = release_array_twoD_arrayI(this->last_down_time, this->node_num);

	if(this->DSD )
		this->DSD = release_array_threeD_arrayF(this->DSD, this->node_num,  this->node_num);
	if(this->estimatedIDSD )
		this->estimatedIDSD = release_array_threeD_arrayF(this->estimatedIDSD, this->node_num, this->node_num);
	if(this->estimatedHIDSD )
		this->estimatedHIDSD = release_array_threeD_arrayF(this->estimatedHIDSD, this->node_num, this->node_num);
	if(this->temporalCorrelations )
		this->temporalCorrelations = release_array_twoD_arrayF(this->temporalCorrelations, this->node_num);
	if(this->distances )
		this->distances = release_array_threeD_arrayF(this->distances, this->node_num, this->node_num);
	if(this->average_distances )
		this->average_distances = release_array_twoD_arrayF(this->average_distances, this->node_num);
	if(this->std_distances )
		this->std_distances = release_array_twoD_arrayF(this->std_distances, this->node_num);
	if(this->nodePathLengths )
		this->nodePathLengths = release_array_twoD_arrayF(this->nodePathLengths, this->node_num);
	if(this->nodesPathLengths )
		free(this->nodesPathLengths);
	if(this->nodeTripLengths )
		this->nodeTripLengths = release_array_twoD_arrayF(this->nodeTripLengths, this->node_num);
	if(this->nodesTripLengths )
		free(this->nodesTripLengths);
	if(this->nodePauseTimes )
		this->nodePauseTimes = release_array_twoD_arrayF(this->nodePauseTimes, this->node_num);
	if(this->nodesPauseTimes )
		free(this->nodesPauseTimes);
	if(this->nodeSpeeds )
		this->nodeSpeeds = release_array_twoD_arrayF(this->nodeSpeeds, this->node_num);
	if(this->nodesSpeeds )
		free(this->nodesSpeeds);
	if(this->nodeAngles )
		this->nodeAngles = release_array_twoD_arrayF(this->nodeAngles, this->node_num);
	if(this->nodesAngles )
		free(this->nodesAngles);

	if(this->cells != NULL)
		this->cells = release_array_twoD_arrayI(this->cells, (this->node_num +1));
	if(this->ld_pdf != NULL)
		free(this->ld_pdf);
	if(this->nd_pdf != NULL)
		free(this->nd_pdf);
	if(this->static_pdf != NULL)
		free(this->static_pdf);

	if(this->ini_x != NULL)
		free(this->ini_x);
	if(this->ini_y != NULL)
		free(this->ini_y);

}

void Trace::calculateMetrics(){

	this->metrics[4] = this->improved_degree_of_spatial_dependence();
	this->metrics[5] = this->improved_degree_of_temporal_dependence();
	this->metrics[7] = this->degree_of_node_proximity();
	this->metrics[0] = this->average_tripLength();
	this->metrics[3] = this->angle_variation_coefficient();
	this->metrics[1] = this->speed_variation_coefficient();
	this->metrics[2] = this->speed_angle_rate();
	this->metrics[8] = this->degreeOfSpatialDistribution();
	this->metrics[6] = this->degreeOfSpatialAcessibility();
	this->metrics[9] = this->degreeOfLinkChanges();
	this->metrics[10] = this->metrics[6];
/*
	metrics[0] //ATL
	metrics[1] //SVC
	metrics[2] //SAR
	metrics[3] //AVC
	metrics[4] //IDSD
	metrics[5] //IDTD
	metrics[6] //DSA
	metrics[7] //DNP
	metrics[8] //DNSD
	metrics[9] //DLC
	metrics[10] //DSA
*/
}
void Trace::printMetrics(){

	std::cout << '\n' << this->metrics[0] <<" ATL" << '\n';
	std::cout << this->metrics[1] << " SVC" << '\n';
	std::cout << this->metrics[2] << " SAR" << '\n';
	std::cout << this->metrics[3] << " AVC" << '\n';
	std::cout << this->metrics[4] << " IDSD" << '\n';
	std::cout << this->metrics[5] << " IDTD" << '\n';
	std::cout << this->metrics[6] << " DSA" << '\n';
	std::cout << this->metrics[7] << " DNP" << '\n';
	std::cout << this->metrics[8] << " DNSD" << '\n';
	std::cout << this->metrics[9] << " DLC" << '\n';
	std::cout << this->metrics[10] << " DSA" << '\n';
	std::cout << "-----------------"<< '\n';
}

data ** Trace::release_array_twoD_arrayD(data **m, int cols){

	for (int ij = 0; ij < cols; ij++){
		if(m[ij])
			free(m[ij]);
	}
	free(m);

	return NULL;
}

int ** Trace::release_array_twoD_arrayI(int **m, int cols){

	for (int ij = 0; ij < cols; ij++){
		if(m[ij])
			free(m[ij]);
	}
	free(m);

	return NULL;
}

double ** Trace::release_array_twoD_arrayF(double **m, int cols){

	for (int ij = 0; ij < cols; ij++){
		if(m[ij])
			free(m[ij]);
	}
	free(m);

	return NULL;
}

double *** Trace::release_array_threeD_arrayF(double ***m, int rows, int cols){

	for (int ii = 0; ii < rows; ii++) {
		for (int ij = 0; ij < cols; ij++) {
			free(m[ii][ij]);
		}
			free(m[ii]);
	}
	free(m);
		return NULL;
}

int ** Trace::twoD_array_allocI(int x, int y){ 																				// dynamic alloc for bi-dimensional matrices

	int ** matrix = (int **) malloc(x * sizeof(int *));

	for (int i = 0; i < x; ++i)
			matrix[i] = (int *) malloc(y * sizeof(int));

	return matrix;
}

double ** Trace::twoD_array_allocF(int x, int y){ 																				// dynamic alloc for bi-dimensional matrices

	double ** matrix = (double **) malloc(x * sizeof(double *));
	for (int i = 0; i < x; ++i)
			matrix[i] = (double *) malloc(y * sizeof(double));

	return matrix;
}

double *** Trace::threeD_array_allocF(int x, int y, int z){															// dynamic alloc for three-dimensional matrices

	double *** matrix = (double ***) malloc(x * sizeof(double ***));

	for(int i = 0; i < x; ++i){
			matrix[i] =  (double **) malloc(y * sizeof(double**));
			for(int j = 0; j < y; ++j){
				matrix[i][j] = (double *) malloc(z * sizeof(double));
		}
	}

	return matrix;
}

void Trace::printTrace(data **m, int i, int j){

	for (int ki = 0; ki < i; ki++) {
		for (int kj = 0; kj < j;kj++) {
			printf("[%d][%d]\n time: %lf\n id: %d\n start: %lf\n x/y: %lf/%lf \n nX/nY: %lf/%lf\n angle: %lf\n speed: %lf\n timetoD: %d\n --------------------\n",
							ki,kj,trace[ki][kj].time,  trace[ki][kj].id, trace[ki][kj].start, trace[ki][kj].x, trace[ki][kj].y,  trace[ki][kj].next_x,  trace[ki][kj].next_y,  trace[ki][kj].angle,  trace[ki][kj].speed,  trace[ki][kj].timetodestiny);
		}
	}
}

double Trace::mean(int* pdf){
	int i;
	int sum1=0,sum2=0;
	double avg;

	for (i=0; i<time_slot+1;i++){
		sum1 = sum1 + pdf[i];
		sum2 = sum2 + i*pdf[i];
	}

	avg= (double)sum2/(double)sum1;
	return avg;
}

void Trace::initiate(){
	int i,j,t;

	this->total_link_change=0;

	for(i=0;i<this->node_num;i++)
		for(j=0;j<this->node_num;j++){

			this->link_status[i][j]=-1;
			this->link_up_num[i][j]=0; this->link_down_num[i][j]=0;
			this->last_up_time[i][j]=-1; this->last_down_time[i][j]=-1;

			//average and std of distances between nodes
			this->average_distances[i][j] = 0.0;
			this->std_distances[i][j] = 0.0;
			//correlations and distances historic
			for (t=0; t<time_slot; t++) {
				this->DSD[i][j][t]=0.0;
				this->estimatedIDSD[i][j][t]=0.0;
				this->estimatedHIDSD[i][j][t]=0.0;
				this->temporalCorrelations[i][t]=0.0;
				this->distances[i][j][t]=0.0;
			}
		}
	for(i=0;i<this->time_slot+1;i++)
		ld_pdf[i]=0;

	setAngleSpeedArrays();
}

void Trace::setAngleSpeedArrays(){

	//change the default array value (0 to NIL), since 0 has a meaning for both speed and angle.
	for (int id = 0; id < this->node_num; id++) {
		for (int k = 0; k < this->node_angle_slot;k++) {
			this->nodeAngles[id][k] = NIL;
		}

		for (int k = 0; k < this->node_speed_slot; k++) {
			this->nodeSpeeds[id][k] = NIL;
		}
		for (int k = 0; k < this->node_trip_length_slot; k++) {
			this->nodeTripLengths[id][k] = NIL;
		}

		for (int k = 0; k < this->node_path_length_slot; k++) {
			this->nodePathLengths[id][k] = NIL;
		}

	}

	for (int k = 0; k < this->nodes_angle_slot; k++) {
		this->nodesAngles[k] = NIL;
	}


	for (int k = 0; k < this->nodes_speed_slot; k++) {
		this->nodesSpeeds[k] = NIL;
	}

	for (int k = 0; k < this->nodes_trip_length_slot; k++) {
		this->nodesTripLengths[k] = NIL;
	}

	for (int k = 0; k < this->nodes_path_length_slot; ++k) {
		this->nodesPathLengths[k] = NIL;
	}

}

//=-----
double Trace::degreeOfLinkChanges(){
	unsigned long long int n = this->node_num;
	unsigned long long int max;

	max = factorial(n) / (factorial(2) * (n - 2));

	return log(this->total_link_change)/ log((double)max);
}
//=-----


void Trace::read_trace(const char *filename){
	FILE *trace_fp;
	int i=0;

	std::ofstream batch_file;
	batch_file.open("erros.txt", std::ofstream::out | std::ofstream::app);

	char word[20],sentence[100],temp,c0,c1;
	int id = 0;
	int t = 0;
	double time,x,y,speed;
	double x0,y0,z0;

 	trace_fp = fopen(filename,"r");
	if(trace_fp == NULL){
		printf("CANNOT find TRACE file!\n");
		exit(0);
	}

	while(!feof(trace_fp)){
		temp=fgetc(trace_fp);
		if(temp=='#')	//if the comment is '#', skip it
		{	if(fgets(sentence,100,trace_fp)==NULL)
				printf("Wrong format for # comment!\n");
		}
		else if(temp=='$'){	//if not comment, analyze it
			//Check the two initial letters
			c0=fgetc(trace_fp);
			c1=fgetc(trace_fp);
			// $node(i) set X_ 26.523097872900
			if(c0=='n' && c1=='o'){
				for(i=0;i<4;i++)
					temp=fgetc(trace_fp);
				fscanf(trace_fp,"%d",&id);
				temp=fgetc(trace_fp);
				fscanf(trace_fp,"%s",word);
				fscanf(trace_fp,"%s",word);

				if(word[0]=='X'){
					fscanf(trace_fp,"%lf",&x0);
					ini_x[id]=x0;
					trace[id][t].x=x0;
					if(trace[id][t].x >= (this->SCENARIO_WIDTH+1) || trace[id][t].x <= -1){
						std::cout << "point out of the scenario: " << "("<< trace[id][t].x << "," << trace[id][t].y << ")"<< '\n';
						exit(0);
					}
				}
				else if(word[0]=='Y'){
					fscanf(trace_fp,"%lf",&y0);
					ini_y[id]=y0;
					trace[id][t].y=y0;
					if(trace[id][t].y >= (this->SCENARIO_LENGTH+1) || trace[id][t].y <= -1){
						std::cout << "point out of the scenario: " << "("<< trace[id][t].x << "," << trace[id][t].y << ")"<< '\n';
						exit(0);
					}
				}
				else{
				  fscanf(trace_fp,"%lf",&z0);
				}
			}
			// $god_ set-dist 0 1 1677215
			else if(c0=='g' && c1=='o'){
				if(fgets(sentence,100,trace_fp)==NULL)
					printf("Wrong format for $god_ argument!\n");
				}
			// $ns_ at 30.000000234323 "$node_(1) setdest 534.67642310 435.43899348 43.367834743"
			// or $ns_ at 344.442322520850 "$god_ set-dist 0 1 7215"
			else if(c0=='n' && c1=='s'){
				temp=fgetc(trace_fp);
				fscanf(trace_fp,"%s",word);
				fscanf(trace_fp,"%lf",&time);

				t=(int)time;



				temp=fgetc(trace_fp);
				temp=fgetc(trace_fp);
				temp=fgetc(trace_fp); //< "$>

				c0=fgetc(trace_fp);
				if(c0=='n'){
					for(i=0;i<5;i++)
						temp=fgetc(trace_fp);
					fscanf(trace_fp,"%d",&id);
					temp=fgetc(trace_fp);

					fscanf(trace_fp,"%s",word);
					fscanf(trace_fp,"%lf",&x);
					fscanf(trace_fp,"%lf",&y);
					fscanf(trace_fp,"%lf",&speed);
					temp=fgetc(trace_fp);
					trace[id][t].id=id;
					trace[id][t].time=time;
		//		printf("TIME cel: %lf \n", trace[id][t].time);
					trace[id][t].next_x=x; //x aqui eh o x de destino (prox parada, q o no atingira apos t segundos) nao necessariamente o proximo x!
					trace[id][t].next_y=y;
					if(trace[id][t].x <= -1 || trace[id][t].y <= -1 || trace[id][t].x >= (this->SCENARIO_WIDTH+1) || trace[id][t].y >= (this->SCENARIO_LENGTH +1)){
						std::cout << "point out of the scenario: " << filename << " ("<< trace[id][t].next_x << "," << trace[id][t].next_y << ")" <<'\n';
						exit(0);
					}
					trace[id][t].speed=speed;
					trace[id][t].angle=0;

					if(t==0){ //find the velocity angle and the duration of the first trip
						trace[id][t].x = ini_x[id];
						trace[id][t].y = ini_y[id];
						if(trace[id][t].x <= -1 || trace[id][t].y <= -1 || trace[id][t].x >= (this->SCENARIO_WIDTH+1) || trace[id][t].y >= (this->SCENARIO_LENGTH +1)){
							std::cout << "point out of the scenario: " << "("<< trace[id][t].x << "," << trace[id][t].y << ")"<< '\n';
							exit(0);
						}

						trace[id][t].angle = get_angle(trace[id][t].x, trace[id][t].y, trace[id][t].next_x, trace[id][t].next_y);
						trace[id][t].timetodestiny = (int) dist(trace[id][t].x, trace[id][t].y, trace[id][t].next_x,trace[id][t].next_y) / trace[id][t].speed;
					}
				}
				else if(c0=='g')
				{	if(fgets(sentence,100,trace_fp)==NULL)
						printf("Wrong format for $ns $god_ argument!\n");
				}
			}// for "ns","node","god"
		}//for "# or $"
	}//for while
	//is_open
	fclose(trace_fp);
}

void Trace::set_data(){

	int id,t,t_aux,ttd=0;
	double distance;

	//waypoints: a bidimensional array that stores the visiting points of a node

	waypoints = new struct point* [node_num];
	for(int i=0;i<node_num;i++)
		waypoints[i]=new struct point[this->max_number_Waypoints];

	for(id=0;id<node_num;id++){
		//add the first waypoint (at t=0)
		waypoints[id][0].x=trace[id][0].x;
		waypoints[id][0].y=trace[id][0].y;
		waypoints[id][0].time=0;
		t=t_aux=0;
		while (this->node_is_stationary(id,t_aux) && t_aux<time_slot){
			this->update_stationary_state(id,t_aux);
			t_aux++;
			//printSingleTrace(id,t_aux);
		}
		t = t_aux;

		// printf("\nNS00 %f",this->nodeSpeeds[0][0]);
		int pindex = 0; //pause time array index
		int aindex = 0; //velocity angle array index
		int sindex  = 0; //speed array index
		int tripindex = 0;	//trip length array index
		int windex = 1; //waypoint index
		int wtindex = 0; //waypoint time index (the time steps when nodes start to pause)

		//if t>0, implies that node has been paused for 't' seconds
		if (t > 0)
			this->nodePauseTimes[id][pindex++]=t;

		int paux = 0;

		while(t < this->time_slot){
			// printf("t on set_data %d\n", t);
			// printf("id on set_data %d\n", id);
			// printf("TIME cel: %lf \n", trace[id][t].time);

			ttd = this->start_new_movement(id,t); //ttd is the Time To arrive at the next Destination

			if (aindex == 0) { //the first value must be stored

				this->nodeAngles[id][aindex++] = this->getPositiveAngle(trace[id][t].angle);

			} //only store node's velocity angle iff it's different than the previous value
			else if (t>0 && !this->equal_or_almost_equal(trace[id][t].angle,trace[id][t-1].angle)){

				this->nodeAngles[id][aindex++] = this->getPositiveAngle(trace[id][t].angle); //1..360 degrees (instead of using radians)
			}


			this->nodeSpeeds[id][sindex++] = trace[id][t].speed;
			// std::cout << "708" << '\n';
			// this->verificaTrace(trace);
			for (t_aux = t+1; ((ttd > 0) && t_aux < this->time_slot); t_aux++) {
				ttd = this->update_moving_state(id,t_aux);
			}
			// std::cout << "713" << '\n';
			// this->verificaTrace(trace);
			if (t_aux >= this->time_slot){
				break;
			}
			t_aux--; //in order to check if node was previously stationary

			paux=t_aux;

			if (node_is_stationary(id,t_aux)){

				//add one more waypoint
				waypoints[id][windex].x=trace[id][t_aux].x;
				waypoints[id][windex].y=trace[id][t_aux].y;
				waypoints[id][windex].time=t_aux;

				//store the distance between this and the previous waypoint
				nodeTripLengths[id][tripindex] = dist(waypoints[id][windex].x, waypoints[id][windex].y,
													waypoints[id][windex-1].x, waypoints[id][windex-1].y);

				//now compute all the distance traveled between this and the previous waypoint
				nodePathLengths[id][tripindex++] = this->distanceTraveled(id, waypoints[id][windex-1].time, waypoints[id][windex].time);
				windex++;
			}

			while (this->node_is_stationary(id,t_aux) && t_aux < this->time_slot){
				this->update_stationary_state(id,t_aux);
				t_aux++;
			}

			//set the pause time duration
			if (trace[id][t_aux-1].speed==0){ //check if node was paused one time step before
				if (t_aux>paux){
					this->nodePauseTimes[id][pindex++]=t_aux-paux;
				}
			}
			if (t_aux >= this->time_slot){
				break;
			}
			t=t_aux;
		}
	}
	this->fillNodesSpeeds();
	this->fillNodesPauseTimes();
	this->fillNodesAngles();
	this->fillDistances();
	this->fillNodesTripLengths();
	this->fillNodesPathLengths();
}

double Trace::distanceTraveled(int id, int t1, int t2){ //t2>t1 | //all the distance traveled between two time steps (measuring step by step)

	double x1,y1;
	double x2,y2;
	double distx,disty;
	double alldistance = 0;

	for (int t = t1+1; t <= t2; t++) {
		x1 = trace[id][t-1].x;
		y1 = trace[id][t-1].y;
		x2 = trace[id][t].x;
		y2 = trace[id][t].y;
		distx = x2-x1;
		disty = y2-y1;
		alldistance += sqrt(distx*distx+disty*disty);
	}

	return alldistance;
}

double Trace::getPositiveAngle(double a){

	double angle = (180 * a)/PI;

	return angle > 0 ? angle : 360 - (-1)*angle;

}

void Trace::fillNodesPauseTimes(){

	int aux=0, k=0;
	for (int i = 0; i < this->node_num; i++) {
		k=0;
		while (this->nodePauseTimes[i][k] > 0 && aux < this->nodes_pause_time_slot){
			this->nodesPauseTimes[aux++] = this->nodePauseTimes[i][k];
			k++;
		}
	}
}

void Trace::fillNodesSpeeds(){

	int aux=0, k=0;


	for (int i = 0; i < this->node_num; ++i) {
		k=0;

		while (this->nodeSpeeds[i][k] != NIL && aux < this->nodes_speed_slot){
			this->nodesSpeeds[aux++] = this->nodeSpeeds[i][k];
			k++;

		}
	}

}

void Trace::fillNodesAngles(){

	int aux=0, k=0;

	for (int i = 0; i < this->node_num; ++i) {
		k=0;
		while (this->nodeAngles[i][k] != NIL && aux < this->nodes_angle_slot){
			this->nodesAngles[aux++] = this->nodeAngles[i][k];
			k++;
		}
	}
}

void Trace::fillNodesTripLengths(){

	int aux=0, k=0;

	for (int i = 0; i < this->node_num; ++i) {
		k=0;
		while (this->nodeTripLengths[i][k] != NIL && aux < this->nodes_trip_length_slot){
			this->nodesTripLengths[aux++] = this->nodeTripLengths[i][k];
			k++;
		}
	}
}

void Trace::fillNodesPathLengths(){

	int aux=0, k=0;

	for (int i = 0; i < this->node_num; ++i) {
		k=0;
		while (this->nodePathLengths[i][k] != NIL && aux < this->nodes_path_length_slot){
			this->nodesPathLengths[aux++] = this->nodePathLengths[i][k];
			k++;
		}
	}
}

void Trace::fillDistances(){

	for(int i=0; i < this->node_num;i++){
		for(int j=i+1; j < this->node_num;j++){
			for(int t=0;t < this->time_slot;t++){
				this->distances[i][j][t] = this->distance_i_j(i,j,t);
				//if (i==0 && j==1)
				//printf("distances[%d][%d][%d]= %f\n", i, j, t, distances[i][j][t]);
			}
		}
	}
}

double Trace::speed_angle_rate(int i){

	int numberOfSpeedValues = this->getLength(this->nodeSpeeds[i], this->node_speed_slot);
	int numberOfAngleValues = this->getLength(this->nodeAngles[i], this->node_angle_slot);

	int MAX = this->node_speed_slot;
	if (this->node_angle_slot > this->node_speed_slot){
		MAX = this->node_angle_slot;
	}

	if (numberOfAngleValues > 0){
		return (double)numberOfSpeedValues / (double)numberOfAngleValues;
	} else
		return 0; //should never happen
}

double Trace::speed_angle_rate(){

	int numberOfSpeedValues = this->getLength(this->nodesSpeeds, this->nodes_speed_slot);
	int numberOfAngleValues = this->getLength(this->nodesAngles, this->nodes_angle_slot);

	int MAX = this->nodes_speed_slot;
	if (this->nodes_angle_slot > this->nodes_speed_slot){
		MAX = nodes_angle_slot;
	}

	if (numberOfAngleValues > 0){
		return (double)numberOfSpeedValues / (double)numberOfAngleValues;
	} else
		return 0; //should never happen
}

double Trace::angle_variation_coefficient(){

	double mean = getAverageNotZero(this->nodesAngles, this->nodes_angle_slot);
	double std = getStdNotZero(this->nodesAngles, mean, this->nodes_angle_slot);

	return std/mean;
}

double Trace::speed_variation_coefficient(){

	double mean = getAverageNotZero(this->nodesSpeeds, this->nodes_speed_slot);
	//printf("mean=%f\n",mean);
	double std = getStdNotZero(this->nodesSpeeds, mean, this->nodes_speed_slot);
	//printf("std=%f\n",std);
	return std/mean;
}

double Trace::pause_variation_coefficient(){

	double mean = getAverageNotZero(this->nodesPauseTimes, this->nodes_pause_time_slot);
	double std = getStdNotZero(this->nodesPauseTimes, mean, this->nodes_pause_time_slot);

	return std/mean;
}

int Trace::getLength(double array[], int size){

	int count = 0;
	for (int k = 0; (k < size) && (array[k] != NIL); k++) {
		count++;
	}

	return count;
}

void Trace::printPauseTimes(int i){

	printf("Pause times of node %d:\n",i);
	for(int k=0; k < this->node_pause_time_slot; k++){
		printf("%f \t", this->nodePauseTimes[i][k]);
	}
}

void Trace::printPauseTimes(){

	printf("Pause times of all nodes:\n");
	for(int k=0;k < this->nodes_pause_time_slot; k++){
		printf("%f \t", this->nodesPauseTimes[k]);
	}
}

void Trace::printAngles(int i){

	printf("Velocity angles of node %d:\n",i);
	for(int k=0; this->nodeAngles[i][k] != NIL && k < this->node_angle_slot;k++){
		printf("%f \n", this->nodeAngles[i][k]);
	}
}

void Trace::printAngles(){

	printf("Velocity angles of all node:\n");
	for(int k=0; this->nodesAngles[k] != NIL && k < this->nodes_angle_slot;k++){
		printf("%f \n", this->nodesAngles[k]);
	}
}

void Trace::printSpeeds(int i){

	printf("Speeds of node %d:\n",i);
	for(int k=0; this->nodeSpeeds[i][k] != NIL && k < this->node_speed_slot ;k++){
		printf("%f \n", this->nodeSpeeds[i][k]);
	}
	printf("saindo printSpeeds\n");
}

void Trace::printSpeeds(){

	printf("Speeds of all nodes:\n");
	for(int k=0; this->nodesSpeeds[k] != NIL && k < this->nodes_speed_slot; k++){
		printf("%f \n", this->nodesSpeeds[k]);
	}

}

void Trace::printTripLengths(int i){

	printf("Trip lengths for node %d:\n",i);
	for(int k=0; this->nodeTripLengths[i][k] != NIL && k < this->node_trip_length_slot; k++){
		printf("%f \n", this->nodeTripLengths[i][k]);
	}
}

void Trace::printTripLengths(){

	printf("Trip lengths for all nodes:\n");
	for(int k=0; this->nodesTripLengths[k] != NIL && k < this->nodes_trip_length_slot; k++){
		printf("%f \n", this->nodesTripLengths[k]);
	}
}

void Trace::printPathLengths(int i){

	printf("Path lengths for node %d:\n",i);
	for(int k=0; this->nodeTripLengths[i][k] != NIL && k < this->node_trip_length_slot; k++){
		printf("Path[%d] %f \t Trip[%d] % f \n", k, this->nodePathLengths[i][k], k, this->nodeTripLengths[i][k]);
	}
}

void Trace::printPathLengths(){

	printf("Path lengths for all nodes:\n");
	for(int k=0; this->nodesPathLengths[k] != NIL && k < this->nodes_trip_length_slot; k++){
		//printf("Path[%d] %f \t Trip[%d] % f \n", k, nodesPathLengths[k], k, nodesTripLengths[k]);
		printf("%f \n", this->nodesPathLengths[k]);
	}
}

void Trace::printSingleTrace(int i, int t){

	printf("trace[%d][%d] (%f,%f) (%f %f) %f %f \n",i,t,trace[i][t].x,trace[i][t].y,
			trace[i][t].next_x,trace[i][t].next_y,
			trace[i][t].speed,trace[i][t].angle/3.14159265359*180);
}

int Trace::start_new_movement(int id, int t){

	//starting a new trip: actual x,y should be the previous next_x, next_y
	if (t > 0){
		trace[id][t].x = trace[id][t-1].next_x;
		trace[id][t].y = trace[id][t-1].next_y;
	}

	double distance = dist(trace[id][t].x,trace[id][t].y,trace[id][t].next_x,trace[id][t].next_y);

	trace[id][t].id=id;
	trace[id][t].time=t;
	trace[id][t].angle = get_angle(trace[id][t].x,trace[id][t].y,trace[id][t].next_x,trace[id][t].next_y);
	trace[id][t].timetodestiny = (int) distance / trace[id][t].speed + 1; //due the int is truncated during double conversion
	return trace[id][t].timetodestiny;
}

void Trace::update_stationary_state(int id, int t){

	if(t==0){ //there is x,y but not next_x,y
		trace[id][t].next_x = trace[id][t].x;
		trace[id][t].next_y = trace[id][t].y;
	}
	 else{ //there is next_x,y but not x,y
		trace[id][t].x=trace[id][t-1].next_x;
		trace[id][t].y=trace[id][t-1].next_y;
	}
	trace[id][t].id=id;
	trace[id][t].time=t;
	trace[id][t].next_x=trace[id][t].x;
	trace[id][t].next_y=trace[id][t].y;
	trace[id][t].speed = 0.0; //correction if the trace was wrong (like in the SLAW code, where it is 1.0)
	trace[id][t].angle = 0.0;
	trace[id][t].timetodestiny = 0.0; //of course, node is not moving!
}

int Trace::update_moving_state(int id, int t){

	//update new x,y values
	trace[id][t].id=id;
	trace[id][t].time=t;
	trace[id][t].x = trace[id][t-1].x + trace[id][t-1].speed*cos(trace[id][t-1].angle);// could have a segmentation fault here [t-1], but for some reason when t=0 the funciont inst called (maybe because this: $node_(id) set X_ xx.xxxxx \n $node_(id) set Y_ yy.yyyyy)
	//printf("trace[id][t].x: %d \n",trace[id][t].x);
	trace[id][t].y = trace[id][t-1].y + trace[id][t-1].speed*sin(trace[id][t-1].angle);// could have a segmentation fault here [t-1], but for some reason when t=0 the funciont inst called (maybe because this: $node_(id) set X_ xx.xxxxx \n $node_(id) set Y_ yy.yyyyy)
	trace[id][t].x = floor(trace[id][t].x * 100 +  0.5) / 100; //to avoid very small decimal values
	//printf("trace[id][t].x: %d \n",trace[id][t].x);
	trace[id][t].y = floor(trace[id][t].y * 100 +  0.5) / 100; //to avoid very small decimal values

//--- the block above prevents that no node will be placed out of the scenario borders
	if(trace[id][t].y > this->SCENARIO_LENGTH)
		trace[id][t].y = this->SCENARIO_LENGTH;
	if(trace[id][t].x > this->SCENARIO_WIDTH)
		trace[id][t].x = this->SCENARIO_WIDTH;
	if(trace[id][t].y < 0)
		trace[id][t].y = 0.0;
	if(trace[id][t].x < 0)
		trace[id][t].x = 0.0;

	//BUG AQUI: pode acontecer de o no ja ter chegado quando ttd=2! ex: no 0 em t=68,95 (RPGM_d900_n10_x500_y500_a5_s0_r100_c0.0_l1_h10_p0_0)
	if(trace[id][t-1].timetodestiny == 1 || (trace[id][t-1].timetodestiny == 2 && trace[id][t].next_x != 0)){
		//node arrived at destination at time t, then actual position should be the same as next position at t-1.
		trace[id][t].x = trace[id][t-1].next_x;
		trace[id][t].y = trace[id][t-1].next_y;
		trace[id][t].timetodestiny = 0;

		//if next_x,y is not zero than it should be equal to x,y (node is stationary now)
		if (trace[id][t].next_x == 0){
			trace[id][t].next_x = trace[id][t].x;
			trace[id][t].next_y = trace[id][t].y;
		}
	}
	else{ //node is still moving towards destination, then we should update the speed, angle and timetodestiny variables

		trace[id][t].speed = trace[id][t-1].speed;
		trace[id][t].angle = trace[id][t-1].angle;
		trace[id][t].timetodestiny = trace[id][t-1].timetodestiny - 1;

		if (trace[id][t].next_x == 0 && trace[id][t].next_y == 0){ //i.e., there is no data about node id at time t in the trace file
			trace[id][t].next_x = trace[id][t-1].next_x;
			trace[id][t].next_y = trace[id][t-1].next_y;
		}

		/*if(is_at_destination(id,t)){ //node finally arrives at destination!
			trace[id][t].speed = 0.0; //SLAW FIX
			trace[id][t].angle = 0.0;
			trace[id][t].timetodestiny = 0; //maybe this is redundant
		}*/
	}

	return trace[id][t].timetodestiny;
}

bool Trace::node_is_stationary(int id, int t){
	//initial stationary condition:
	if (t==0 and trace[id][t].speed==0){
		trace[id][t].next_x=trace[id][t].x;
		trace[id][t].next_y=trace[id][t].y;
		return true;
	}
	//node is stationary if previous speed is 0 and next_x,y is equal as previous x,y or next_x,y is 0

	return (is_at_destination(id,t) || trace[id][t].speed==0 ||
			(equal_or_almost_equal(trace[id][t-1].x,trace[id][t].next_x) && equal_or_almost_equal(trace[id][t-1].y,trace[id][t].next_y)));

	/*bool previous_paused = trace[id][t-1].speed == 0;
	return (   (previous_paused && (no_next_xy(id,t) || same_destination(id,t)) )
			 || is_at_previous_position(id,t) || is_at_destination(id,t)        );*/
}

bool Trace::is_at_previous_position(int id, int t){ //TO CHECK: IS THIS EVER USED ??
	return equal_or_almost_equal(trace[id][t].x, trace[id][t-1].x) &&
			equal_or_almost_equal(trace[id][t].y, trace[id][t-1].y);
}

bool Trace::is_at_destination(int id, int t){ //TO CHECK: IS THIS EVER USED ??
	return equal_or_almost_equal(trace[id][t].x, trace[id][t].next_x) &&
			equal_or_almost_equal(trace[id][t].y, trace[id][t].next_y);
}

bool Trace::same_destination(int id, int t){
	return equal_or_almost_equal(trace[id][t].next_x, trace[id][t-1].next_x) &&
			equal_or_almost_equal(trace[id][t].next_y, trace[id][t-1].next_y);
}

bool Trace::no_next_xy(int id, int t){
	return trace[id][t].next_x==0 && trace[id][t].next_y==0;
}

void Trace::show_trace(){
	int i,t;

	for(i=0;i<node_num;i++)
	{
		for(t=0;t<time_slot;t++)
		{
			printf("trace[%d][%d] (%f,%f) (%f %f) %f %f \n",i,t,trace[i][t].x,trace[i][t].y,
								trace[i][t].next_x,trace[i][t].next_y,
								trace[i][t].speed,trace[i][t].angle/3.14159265359*180);
		}
	}
}

void Trace::show_trace(int start, int end){//shows the trace info from t:0 to lastTimestep

	int i,t;

	for(i=0;i<node_num;i++)
	{
		for(t=start;t<end;t++)
		{
			printf("trace[%d][%d] (%f,%f) (%f %f) %f %f \n",i,t,trace[i][t].x,trace[i][t].y,
								trace[i][t].next_x,trace[i][t].next_y,
								trace[i][t].speed,trace[i][t].angle/3.14159265359*180);
		}
	}
}

void Trace::show_trace(int i, int start, int end){//shows the trace info of node i from t:start to t:end

	for(int t=start;t<end;t++) {
		printf("trace[%d][%d] (%f,%f) (%f %f) %f %f \n",i,t,trace[i][t].x,trace[i][t].y,
							trace[i][t].next_x,trace[i][t].next_y,
							trace[i][t].speed,trace[i][t].angle/3.14159265359*180);
	}
}

void Trace::cal_link(char *filename){
	int t;
	int i,j;
	int current_status;
	std::ofstream fp(filename, std::ofstream::trunc);


	//t=0, initiate the link status;
	for(i=0; i < this->node_num; i++)
		for(j=i+1;j < this->node_num; j++)
		{
			if( dist(trace[i][0].x, trace[i][0].y,
				trace[j][0].x, trace[j][0].y) <= this->RADIUS )
			{	this->link_status[i][j]=1; this->link_status[j][i]=1;
				this->link_up_num[i][j]=1; this->link_up_num[j][i]=1;
				this->last_up_time[i][j]=0; this->last_up_time[j][i]=0;
			}
			else
			{	this->link_status[i][j]=0; this->link_status[j][i]=0;	}
		}

	for(t=1;t < this->time_slot;t++)
	{
		for(i=0;i < this->node_num;i++)
			for(j=i+1;j < this->node_num;j++)
			{
				if( dist(trace[i][t].x, trace[i][t].y,
					trace[j][t].x, trace[j][t].y) <= this->RADIUS)
					current_status = 1;
				else
					current_status = 0;

				// link: 0 ---> 1  link comes up
				if(this->link_status[i][j]==0  && current_status==1)
				{	this->link_up_num[i][j]++; this->link_up_num[j][i]++;
					this->last_up_time[i][j]=t; this->last_up_time[j][i]=t;
					this->link_status[i][j] = current_status;	this->link_status[j][i] = current_status;
				}
				// link: 1 ---> 0 link comes down
				if(this->link_status[i][j]==1 && current_status==0)
				{	this->link_down_num[i][j]++; this->link_down_num[j][i]++;
					this->last_down_time[i][j]=t; this->last_down_time[j][i]=t;
					this->link_status[i][j] = current_status;	this->link_status[j][i] = current_status;
					ld_pdf[this->last_down_time[i][j]-this->last_up_time[i][j]]++;
				}
				//at the end of simulation, count the duration
				if(t==time_slot-1 && this->link_status[i][j]==1 && current_status==1)
					ld_pdf[(this->time_slot - this->last_up_time[i][j])]++;
 			}//for (i,j)
	}//for(t)
	for(i=0;i < this->node_num;i++)
		for(j=i+1; j < this->node_num;j++)
		{
			this->total_link_change += this->link_up_num[i][j];
			this->total_link_change += this->link_down_num[i][j];
		}

	this->avg_link_duration = mean(ld_pdf);

	//
	if(!fp.is_open()){
		printf("can not create the file\n");
	}else{
		for(i=0; i < this->time_slot;i++)
			fp << this->ld_pdf[i] << "\n";
		// fprintf(fp,"%d\n",ld_pdf[i]);
		fp.close();
	}
}

void Trace::cal_node_degree(char* filename){
	int i,j,k;
	int neighbor_all=0;
	int neighbor_slot=0;
	std::ofstream fp(filename, std::ofstream::trunc);


	for(k=0; k < this->time_slot;k++){
		neighbor_slot = 0;
		for(i=0; i < this->node_num;i++)
			for(j=0; j < this->node_num;j++)
			{
				if(i!=j && dist(trace[i][k].x, trace[i][k].y,trace[j][k].x, trace[j][k].y) <= this->RADIUS)
				{	neighbor_all++; neighbor_slot++;}
			}
		this->nd_pdf[k] = (double)neighbor_slot/(double)this->node_num;
	}
	this->node_degree = (double)neighbor_all / (double)(this->time_slot * this->node_num);


	if(!fp.is_open()){
		printf("can not create the file\n");
	}else{
		for(i=0;i < this->time_slot;i++)
			fp << this->nd_pdf[i] << "\n";
		fp.close();
	}
}

void Trace::cal_static_degree(char* filename){
	int i,k;
	int static_all=0;
	int static_slot;
	// FILE *fp;
	std::ofstream fp(filename, std::ofstream::trunc);

	for(k=0; k < this->time_slot; k++){
		static_slot = 0;
		for(i=0; i < this->node_num;i++)
		{
			if(trace[i][k].speed ==0)
			{	static_all++; static_slot++;}
		}
		this->static_pdf[k] = static_slot;
	}
	this->static_degree = (double)static_all / (double)(time_slot);

	if(!fp.is_open()){
		printf("can not create the file\n");
	}else{
		for(i=0;i<time_slot;i++)
			fp << static_pdf[i] << "\n";
		fp.close();
	}
}

double Trace::average_relative_speed(){//REMOTENESS function = STEP function (<2R,1; >2R,0)
	double v1=0.0,v2=0.0,v3=0.0;
	int t,i,j;
	int count=0;

	v3 = 0.0;
	for(t=0; t < this->time_slot;t++)
	{
		v2=0.0; count=0;
		for(i=0; i < this->node_num;i++)
			for(j=i+1; j < this->node_num;j++)
			{
				if(this->distance_i_j(i,j,t) <= 2*this->RADIUS){
					v1 = relative_speed(trace[i][t].speed,trace[i][t].angle,
						trace[j][t].speed,trace[j][t].angle);
					v2 += v1;
					count++;
				}
			}
		if(count!=0)
			v3 = v3 + v2/(double)count;
	}
	return v3 / (double)this->time_slot;
}

double Trace::degree_of_temporal_dependence(){//Degree of Temporal Dependence - DTD
	double cor1_old=0.0,cor2_old=0.0,cor3_old=0.0;
	int id,t,k;
	int count_old=0;
	double DTD = 0.0;

	for(id=0; id < this->node_num; id++){
		cor2_old=0.0; count_old=0;
		for(t=0; t < (this->time_slot-50); t++)
			for(k=t+1;k < (t+50); k++){
				if((k-t) < 50){ //ALWAYS TRUE!!!
					cor1_old = DSDijt(trace[id][t].speed,trace[id][t].angle,
						trace[id][k].speed,trace[id][k].angle);
					cor2_old += cor1_old;
					count_old++;
				}
			}
		if(count_old!=0)
			cor3_old = cor3_old + cor2_old/(double)count_old;
	}
	DTD = cor3_old / (double)this->node_num;
	return DTD;
}

double Trace::improved_degree_of_temporal_dependence(){//Improved Degree of Temporal Dependence - IDTD

	double cor1=0.0,cor2=0.0,cor3=0.0;
	int i,t;
	int count=0;
	double IDTD = 0.0;

	cor3 = 0.0;

	for(i=0; i < this->node_num; i++){
		cor2=0.0; count=0;
		for(t=0; t < this->time_slot; t++){//t=1?
			if(velocity_has_changed(i,t)){
				cor1 = basic_correlation_positive(trace[i][t].speed,trace[i][t].angle,
					trace[i][t-1].speed,trace[i][t-1].angle);
				cor2 += cor1;
				temporalCorrelations[i][t] = cor1; //this is not used yet
				count++;
			}
			//IMPROVEMENT: 12/05/2011
			//else{ //node has uniform movement or is stationary. Then, should consider the last k movements
			//	cor1 = compute_mean_basic_temporal_correlation(i,t-1);
			//}
		}
		if(count!=0)
			cor3 = cor3 + cor2/(double)count;
	}

	IDTD = cor3 / (double)this->node_num;
	return IDTD;

}

double Trace::degree_of_spatial_dependence(){//Degree of Spatial Dependence - DSD

	double cor1=0.0,cor2=0.0,cor3=0.0;
	int t,i,j;
	int count_old=0;

	cor3 = 0.0;
	for(t=0; t < this->time_slot;t++){
		cor2=0.0; count_old=0;
		for(i=0; i < this->node_num; i++)
			for(j=i+1; j < this->node_num; j++){
				if(distance_i_j(i,j,t) <= 2*RADIUS){
					cor1 = DSDijt(trace[i][t].speed,trace[i][t].angle,
						trace[j][t].speed,trace[j][t].angle);
					this->DSD[i][j][t] = cor1;
					cor2 += cor1;
					count_old++;
				}
			}
		if(count_old!=0)
			cor3 = cor3 + cor2/(double)count_old;
	}
	return cor3 / (double)this->time_slot;
}

double Trace::improved_degree_of_spatial_dependence(){//Improved Degree of Spatial Dependence - IDSD
	double cor1=0.0,cor2=0.0,cor3=0.0;
	int t,i,j;
	int count=0;
	double IDSD = 0.0;

	//clean array estimatedIDSD
	for(i=0; i < this->node_num; i++)
		for(j=i+1; j < this->node_num; j++)
			for(t=0; t < this->time_slot; t++)
				this->estimatedIDSD[i][j][t] = 0.0;

	for(t=0; t < this->time_slot;t++){
		cor2=0.0; count=0;

		for(i=0; i < this->node_num; i++)
			for(j=i+1; j < this->node_num; j++){

				if(this->distance_i_j(i,j,t) <= 2*this->RADIUS){

					if (this->is_stopped(i,t) || this->is_stopped(j,t)){

						if (this->TIMEPAUSE>0) {
							cor1 = SMAverage(i,j,t, TIMEPAUSE/10); 														// 0?
						}
						else {
							cor1 = DSDijt(trace[i][t].speed,trace[i][t].angle,
											trace[j][t].speed,trace[j][t].angle);
						}
					}
					else{
						cor1 = DSDijt(trace[i][t].speed,trace[i][t].angle,
							trace[j][t].speed,trace[j][t].angle);
					}

					this->estimatedIDSD[i][j][t] = cor1;
					cor2 += cor1;
					count++;
				}
				else{
					//if distance > 2R
					this->estimatedIDSD[i][j][t] = 0.0;
				}
			}

		if(count!=0)
			cor3 = cor3 + cor2/(double)count;
	}

	IDSD = cor3 / (double)this->time_slot;
	return IDSD;

}

double Trace::high_improved_degree_of_spatial_dependence(){//High Improved Degree of Spatial Dependence - HIDSD

	double cor1=0.0,cor2=0.0,cor3=0.0;
	int t,i,j;
	int count=0;
	double IDSD = 0.0;

	//clean array estimatedHIDSD
		for(i=0; i < this->node_num;i++)
			for(j=i+1; j < this->node_num;j++)
				for(t=0; t < this->time_slot;t++)
					this->estimatedHIDSD[i][j][t] = 0.0;

	//set initial pause time period correlations (undetermined value)
		t=0;
		for(i=0; i < this->node_num;i++)
			for(j=i+1; j < this->node_num;j++)
				while(this->distance_i_j(i,j,t) <= this->RADIUS && this->is_stopped(i,t) && this->is_stopped(j,t)){
					this->estimatedHIDSD[i][j][t] = NIL;
					t++;
				}

	for(t=0; t < this->time_slot; t++){
		cor2=0.0; count=0;

		for(i=0; i < this->node_num; i++){

			for(j=i+1; j < this->node_num; j++){

				if(this->distance_i_j(i,j,t) <= 2*this->RADIUS && this->estimatedIDSD[i][j][t] != NIL){

					//We should remove the initial stationary period since it is impossible to ensure whether there is or no correlation

					if (!this->is_stopped(i,t) && !this->is_stopped(j,t)){
						//CASE 1: both nodes are moving
						cor1 = this->EWMAverage(i,j,t); //we have the actual SD value and past values
					}
					else if (TIMEPAUSE<=0){ //results the same as DSD
						cor1 = DSDijt(trace[i][t].speed,trace[i][t].angle,
									  trace[j][t].speed,trace[j][t].angle);
					}
					else if ( (this->is_stopped(i,t) && !this->is_stopped(j,t)) || (!this->is_stopped(i,t) && this->is_stopped(j,t)) ){
						//CASE 2: one node is moving and the other is stationary
						cor1 = this->WMAverage(i,j,t, this->TIMEPAUSE/10); //we only have past SD values

						//O TIMEPAUSE sera obtido por cada dispositivo e compartilhado, sempre que o veiculo parar o novo valor eh incrementado
						//para calcular o ATP basta dividir o tempo total de pausa pelo tempo em que o sistema foi ligado.
						//TODO: considerar se o no se aproxima ou se afasta do outro? Caso se aproxime aumenta a correlacao, caso se afaste diminui?
					}
					else {
						//CASE 3: both nodes are stationary
						cor1 = this->WMAverage(i,j,t, this->TIMEPAUSE/10);  //we only have past SD values
					}

					this->estimatedHIDSD[i][j][t] = cor1; //keeps the last correlations between the nodes i,j
					if (this->estimatedHIDSD[i][j][t] != NIL){ //considera apenas os valores validos
						cor2 += cor1;
						count++;
					}
				}
				else{
					//if distance > 2R
					this->estimatedHIDSD[i][j][t] = 0.0;
				}
			}
		}

		if(count!=0)
			cor3 = cor3 + cor2/(double)count;
	}

	//int numberofpairs = node_num * (node_num -1) / 2;

	IDSD = cor3 / (double)this->time_slot;

	return IDSD;
}


//This function is only called by IDSD()
double Trace::SMAverage(int i, int j, int t, int periods){//METHOD 1: Simple Moving Average (MSS) (the same as arithmetic mean)


	double cor = 0.0;

	for (int k=1;(k<=periods) && (t-k>=0) && this->estimatedIDSD[i][j][t]!=NIL;k++){
			cor += this->estimatedIDSD[i][j][t-k];
	}

	return cor/periods;
}

//This function is only called by HIDSD()
double Trace::WMAverage(int i, int j, int t, int periods){//METHOD 2: Weighted Moving Average (WMA)

	double cor = 0.0;
	int count = 0;

	//check if nodes are stationary since simulation start; if yes, correlation should be undefined (-99)
	if (this->is_stopped(i,t) && this->is_stopped(j,t) && (t==0 || this->estimatedHIDSD[i][j][t-1] == NIL)){
		return NIL;
	}

	for (int k=0; (k<periods) && (t-k>0); k++){ //vai ao passado ate onde havia correlacao valida
		if (this->estimatedHIDSD[i][j][t-k-1] != NIL){
			cor += (periods-k)* this->estimatedHIDSD[i][j][t-k-1]; //peso decrescente (PA com razao = 1)
			count += periods-k; // N(N+1)/2 soma dos termos de uma PA 1..periods
		}
		else break;
	}

	return count == 0 ? 0.0 : cor/(double)count;
}

//This function is only called by HIDSD()
double Trace::EWMAverage(int i, int j, int t){//METHOD 3: Exponential Weighted Moving Average (EWMA)

	// variante da EWMA, onde alfa pode ser igual a 1/N ou 2/(N+1) (N pode ser o tempo de pausa medio)
	double alfa = 0.125;

	if (t==0 || (this->estimatedHIDSD[i][j][t-1] == NIL)){ //there is no past values
		return DSDijt(trace[i][t].speed,trace[i][t].angle,trace[j][t].speed,trace[j][t].angle);
	}
	else {
		/* Uma variante eh usar DSD(i,j,t-1) [Hunter (1986)] ao inves de DSD(i,j,t) [Roberts (1959)] como valor atual da correlacao.
		 * => http://www.itl.nist.gov/div898/handbook/pmc/section4/pmc431.htm
		 */
		//return (1-alfa)*estimatedHIDSD[i][j][t-1] + alfa*DSDijt(trace[i][t-1].speed,trace[i][t-1].angle,trace[j][t-1].speed,trace[j][t-1].angle);//[Hunter (1986)]
		return (1-alfa)*this->estimatedHIDSD[i][j][t-1] + alfa*DSDijt(trace[i][t].speed,trace[i][t].angle,trace[j][t].speed,trace[j][t].angle);//[Roberts (1959)]
	}
}

double Trace::compute_mean_basic_temporal_correlation(int i, int t){
	double cor = 0.0;
	int iterations = 10; //number of time steps to go back in the history of past movements
	int count = 0;

	for (int k=0; (k < iterations) && (t-k>=0);k++){
		if(velocity_has_changed(i,t-k)){
			cor += this->temporalCorrelations[i][t-k]; //ja foi preenchido esse array???
			count++;
		}
	}

	return count==0 ? 0.0 : cor/(double)count;
}

void Trace::print_spatial_dependence_statistics(int i, int j){
	//based on average_distance() and within_coverage() functions
	double distance = 0.0;
	double all_distance = 0.0;
	int count_coverages = 0;
	int count_both_moving = 0;
	int count_one_moving = 0;
	int count_both_stationary = 0;
	int count_distance_greater2R = 0;

	for(int t=0;t<time_slot;t++){
		distance = dist(trace[i][t].x,trace[i][t].y,
						 trace[j][t].x,trace[j][t].y);
		all_distance += distance;

		if(this->distance_i_j(i,j,t) <= 2*this->RADIUS){

			if (distance <= this->RADIUS){
				count_coverages += 1;
			}

			if (!this->is_stopped(i,t) && !this->is_stopped(j,t)){
				//CASE 1: both nodes are moving
				count_both_moving++;
			}
			else if ( (this->is_stopped(i,t) && !this->is_stopped(j,t)) || (!this->is_stopped(i,t) && this->is_stopped(j,t)) ){
				//CASE 2: one node is moving and the other is stationary
				count_one_moving++;
			}
			else {
				//CASE 3: both nodes are stationary
				count_both_stationary++;
			}

		} else {
			count_distance_greater2R++;
		}

	}
	double average_coverage = (double)count_coverages / (double)this->time_slot;
	double average_distance = all_distance / (double)this->time_slot;

	//magic formula
	printf("count_distance_greater2R = %d\n", count_distance_greater2R);
	printf("count_both_moving = %d\n", count_both_moving);
	printf("count_one_moving = %d\n", count_one_moving);
	printf("count_both_stationary = %d\n", count_both_stationary);

}

void Trace::average_std_distances(){

	//average measurement
	double sum = 0.0;
	for(int i=0; i < this->node_num; i++){
		for(int j=i+1; j < this->node_num; j++){
			sum = 0.0;
			for(int t=0; t < this->time_slot; t++){
				sum += this->distances[i][j][t];
			}
			this->average_distances[i][j] = sum/this->time_slot;
			if (i==0 && j==1)
			printf("average_distances[%d][%d]= %f\n", i, j, this->average_distances[i][j]);
		}
	}
	//std measurement
	double variation = 0.0;
	sum = 0.0;
	for(int i=0; i < this->node_num; i++){
		for(int j=i+1; j < this->node_num; j++){
			for(int t=0; t < this->time_slot; t++){
				variation = this->distances[i][j][t] - this->average_distances[i][j];
				variation = variation*variation;
				sum += variation;
			}
			this->std_distances[i][j] = sqrt(sum/this->time_slot);
			if (i==0 && j==1)
			printf("std_distances[%d][%d]= %f\n", i, j, this->std_distances[i][j]);
		}
	}
}

double Trace::degree_of_node_proximity(){

	double sum_distances_all = 0.0;
	double average_distance_all = 0.0;
	int count = 0;

	for(int i=0; i < this->node_num; i++){
		for(int j=i+1; j < this->node_num; j++){
			sum_distances_all += average_distance(i,j) / this->RADIUS; //distances[i][j] is filled
			count += 1;
		}
	}
	average_distance_all = sum_distances_all / (double)count;
	double DNP = 1 - average_distance_all/this->maximum_average_distance();

	return DNP;
}

double Trace::improved_degree_of_node_proximity(){

	double next_CV = 0.0;
	double sum_CVs = 0.0;
	for(int i=0; i < this->node_num; i++){
		for(int j=i+1; j < this->node_num; j++){
			next_CV = coefficient_of_variation(this->average_distances[i][j], this->std_distances[i][j]);
			sum_CVs += next_CV;
			if (i==2)
			printf("CV[%d][%d]=%f \n",i,j,next_CV);
		}
	}

	return sum_CVs;
}

/*void Trace::printPauseTimes(){

	//joint all pause times durations of all nodes
	double allPauseTimes[nodes_pause_time_slot]; //lembrar de desconsiderar os valores 0, reduzir para array que contenha so valores validos
	int aux=0, j=0;

	for (int i = 0; i < node_num; ++i) {
		j=0;
		while (nodePauseTimes[i][j] > 0 && aux<nodes_pause_time_slot){
			allPauseTimes[aux++] = nodePauseTimes[i][j];
			j++;
		}
	}
	printf("All pause time durations of all nodes:\n");
	for (int k=0; (k<1000 && allPauseTimes[k]>0); ++k) {
		printf("%f \n", allPauseTimes[k]);
	}

}*/

double Trace::maximum_average_distance(){

	return sqrt(this->SCENARIO_LENGTH * this->SCENARIO_LENGTH + this->SCENARIO_WIDTH * this->SCENARIO_WIDTH) / (2*this->RADIUS);
}

void Trace::xy_axis_interval(){////show the x-axis and y-axis interval (min and max)

	double xmin = trace[0][0].x;
	double ymin = trace[0][0].y;
	double xmax = trace[0][0].x;
	double ymax = trace[0][0].y;

	for(int t=0; t < this->time_slot; t++){
		for(int i=0; i < this->node_num; i++){

			if (trace[i][t].x < xmin){
				xmin = trace[i][t].x;
			} else if (trace[i][t].x > xmax){
				xmax = trace[i][t].x;
			}
			if (trace[i][t].y < ymin){
				ymin = trace[i][t].y;
			} else if (trace[i][t].y > ymax){
				ymax = trace[i][t].y;
			}
		}
	}

	printf("x-axis interval=[%f, %f]\n",xmin,xmax);
	printf("y-axis interval=[%f, %f]\n",ymin,ymax);
}

double Trace::average_tripLength(){

	double avg = getAverageNotZero(this->nodesTripLengths, this->nodes_trip_length_slot);
	//double std = getStdNotZero(nodesTripLengths,avg,node_num);

	return avg / (double)this->RADIUS;
}

double Trace::average_pathLength(){

	double avg = getAverageNotZero(this->nodesPathLengths, this->nodes_path_length_slot);
	//double std = getStdNotZero(nodesTripLengths,avg,node_num);

	return avg / (double)this->RADIUS;
}

double Trace::degreeOfSpatialDistribution(){

	int t,i,x,y;

	this->GRID_WIDTH = this->node_num;
	this->GRID_LENGTH = this->node_num;
	this->CELL_WIDTH = this->SCENARIO_WIDTH/(this->GRID_WIDTH);
	this->CELL_LENGTH = this->SCENARIO_LENGTH/(this->GRID_LENGTH);

	//1) initialize the cells matrix
	for (x=0; x < (this->GRID_WIDTH+1); x++){
		for (y=0; y < (this->GRID_LENGTH+1); y++){
			this->cells[x][y]=0; //means no node inside this cell
		}
	}
	//2) fill the cells matrix

	for(t=0; t < this->time_slot; t++){
		for (i=0; i < this->node_num; i++){
			x = trace[i][t].x / this->CELL_WIDTH;
			y =  trace[i][t].y / this->CELL_LENGTH;
			this->cells[x][y]++;
		}
	}

	// printf("1778\n");
	// getchar();
	/*printf("BEFORE PROCESSING: \n");
		printf("LOCATION MATRIX AT TIME %d: \n", t);
		for (x=0; x < GRID_WIDTH; x++){
			for (y=0; y < GRID_LENGTH; y++){
				//printf("cells[%d,%d,%d]=%d",t,x,y,cells[t][x][y]);
				printf("%d	",cells[x][y]);
			}
			printf("\n");
		}
		*/

	int sumX, sumY;

	//3) calculates the sum of all lines and columns of the grid (inserting the values into the last line and column of the grid)

	//filling the last line with the sum of all values
	for (x=0; x < this->GRID_WIDTH; x++){
		sumY=0;
		for (y=0; y < this->GRID_LENGTH; y++){
			sumY += this->cells[x][y];
		}
		this->cells[x][this->GRID_LENGTH] = sumY;
		//if (t>0) cells[t][x][GRID_LENGTH] = cells[t-1][x][GRID_LENGTH] + sumY;
	}
	//filling the last column with the sum of all values
	for (y=0; y < this->GRID_LENGTH; y++){
		sumX=0;
		for (x=0; x < this->GRID_WIDTH; x++){
			sumX += this->cells[x][y];
		}
		this->cells[this->GRID_WIDTH][y] = sumX;
		//if (t>0) cells[t][GRID_WIDTH][y] = cells[t-1][GRID_WIDTH][y] + sumX;
	}

	/*printf("AFTER PROCESSING: \n");
	printf("LOCATION MATRIX AFTER COMPUTATION AT TIME %d: \n", t);
	for (x=0; x <= GRID_WIDTH; x++){
		for (y=0; y <= GRID_LENGTH; y++){
			//printf("cells[%d,%d,%d]=%d",t,x,y,cells[t][x][y]);
			printf("%d	",cells[x][y]);
		}
		printf("\n");
	}
	*/

	//4) calculates the deviation from uniform distribution
	double deviation = 0;
	double deviationSum = 0;

	for (x=0; x < this->GRID_WIDTH; x++){
		deviation = abs(this->cells[x][this->GRID_LENGTH] - this->time_slot);
		deviationSum += deviation;
	}
	for (y=0; y < this->GRID_LENGTH; y++){
		deviation = abs(cells[this->GRID_WIDTH][y] - this->time_slot);
		deviationSum += deviation;
	}

	double MAX_DEVIATION = 4*(this->node_num - 1) * this->time_slot;

	degreeSpatialDistribution = 1 - log(deviationSum)/log(MAX_DEVIATION);

	return degreeSpatialDistribution;

}

double Trace::emptyCells(){

	double emptyCells = 0;
	for (int x=0; x < this->GRID_WIDTH; x++){
		for (int y=0; y < this->GRID_LENGTH; y++){
			if(this->cells[x][y] == 0){
				emptyCells++;
			}
		}
	}

	return emptyCells/(double)(this->GRID_WIDTH * this->GRID_LENGTH);
}
double Trace::degreeOfSpatialAcessibility(){

	double visited = 0;
	for (int x=0; x < this->GRID_WIDTH; x++){
		for (int y=0; y < this->GRID_LENGTH; y++){
			if(cells[x][y] > 0){
				visited++;
			}
		}
	}

	return visited/(double)(this->GRID_WIDTH * this->GRID_LENGTH); // ??
}


//Authors of the paper: 'Feature selection for user motion pattern recognition in mobile networks'
double Trace::positionDensityVariance(){////Position Density Variance metric

	double average = 0;
	double PDV;

	for (int x=0; x < this->GRID_WIDTH; x++){
		for (int y=0; y < this->GRID_LENGTH; y++){
			average += this->cells[x][y];
		}
	}
	average = average/(this->GRID_WIDTH * this->GRID_LENGTH);

	//PDV
}

void Trace::createIdentityVector(int size){
	int vector[this->node_num];
	for (int i = 0; i < this->node_num; ++i) {
		vector[i]=1;
	}
}

void Trace::print_DSD(int i, int j, int start, int end){
	double media = 0.0;
	int count = 0;
	for(int t=start; t<end ;t++){
		printf("%f\n", this->DSD[i][j][t]);
		media += this->DSD[i][j][t];
		count++;
	}
	printf("DSD = %f\n", media/(double)count);
}

void Trace::print_IDSD(int i, int j, int start, int end){
	double media = 0.0;
	int count = 0;
	for(int t=start;t<end;t++){
		printf("%f\n", this->estimatedIDSD[i][j][t]);
		media += this->estimatedIDSD[i][j][t];
		count++;
	}
	printf("IDSD = %f\n", media/(double)count);
}

void Trace::print_HIDSD(int i, int j, int start, int end){
	double media = 0.0;
	int count = 0;
	//printf("%f\n", 0.11111111111111111);
	for(int t=start;t<end;t++){
		printf("%f\n", this->estimatedHIDSD[i][j][t]);
		if (this->estimatedHIDSD[i][j][t] != NIL){ //desconsidera os valores iniciais de pausa (correlation undefined)
			media += this->estimatedHIDSD[i][j][t];
			count++;
		}
	}
	printf("HIDSD = %f\n", media/(double)count);
}

void Trace::print_SpatialMetrics(int i, int j, int start, int end){

	printf("DSD   IDSD   HIDSD\n");
	for(int t=start;t<end;t++){
		printf("%d %f   %f   %f\n", t, this->DSD[i][j][t], this->estimatedIDSD[i][j][t], this->estimatedHIDSD[i][j][t]);
	}
}

void Trace::print_distance_between_nodes(){
	for(int i=0; i < this->node_num;i++){
		for(int j=i+1; j < this->node_num;j++){
			for(int t=0;t < this->time_slot;t++){
				printf("distances[%d][%d][%d]%% = %f\n", i,j,t,this->distances[i][j][t]);
			}
		}
	}
}

double Trace::average_distance(){
	double average_distance_all = 0.0;
	int count = 0;

	for(int i=0; i < this->node_num; i++){
		for(int j=i+1; j < this->node_num; j++){
			average_distance_all += average_distance(i,j) / this->RADIUS;
			count += 1;
		}
	}
	return average_distance_all / (double)count;
}

double Trace::average_distance(int i, int j){
	double distance = 0.0;
	double all_distances = 0.0;
	for(int t=0; t < this->time_slot; t++){
		distance = dist(trace[i][t].x,trace[i][t].y,
						 trace[j][t].x,trace[j][t].y);
		all_distances += distance;
		this->distances[i][j][t] = distance / this->RADIUS;
	}
	return all_distances / (double)this->time_slot;
}

double Trace::average_coverage(){
	int count = 0;
	double average_coverage_all = 0.0;

	for(int i=0; i < this->node_num ; i++){
		for(int j=i+1; j < this->node_num; j++){
			average_coverage_all += this->average_coverage_i_j(i,j);
			count += 1;
		}
	}
	return average_coverage_all / (double)count;
}

double Trace::average_coverage_i_j(int i, int j){
	int count = 0;
	for(int t=0; t < this->time_slot; t++){
		if (dist(trace[i][t].x,trace[i][t].y,
				 trace[j][t].x,trace[j][t].y) <= this->RADIUS){
			count += 1;
		}
	}
	return (double)count / (double)this->time_slot;
}

double Trace::distance_i_j(int i, int j, int t){
	double x0 = trace[i][t].x;
	double y0 = trace[i][t].y;
	double x1 = trace[j][t].x;
	double y1 = trace[j][t].y;

	return dist(x0,y0,x1,y1);
}

bool Trace::stop_trip(int i, int t){//node,time

	if(t==0) return true;

	//node i is stopped at time t iff at time t-1 it were at the same location
	double x0 = trace[i][t-1].x;
	double x1 = trace[i][t].x;
	double y0 = trace[i][t-1].y;
	double y1 = trace[i][t].y;

	double nx1 = trace[i][t].next_x;
	double ny1 = trace[i][t].next_y;

	return (dist(x0,y0,x1,y1)==0.0 || dist(x1,y1,nx1,ny1)==0.0) ? true : false;
}

bool Trace::is_stopped(int i, int t){//node,time

	return (trace[i][t].speed==0) ? true : false;
}

bool Trace::velocity_has_changed(int i, int t){

	if (t==0) return false;

	double speed0 = trace[i][t-1].speed;
	double speed1 = trace[i][t].speed;
	double angle0 = trace[i][t-1].angle;
	double angle1 = trace[i][t].angle;

	if (this->equal_or_almost_equal(speed0,speed1) &&
		this->equal_or_almost_equal(angle0,angle1)){
		return false;
	}
	else {
		return true;
	}
}

bool Trace::equal_or_almost_equal(double x, double y){
	if (x==y){
		return true;
	} else if (x>y) {
		return x-y < 0.01 ? true : false;
	} else {
		return y-x < 0.01 ? true : false;
	}
}

void Trace::test_is_stopped(int x, int y){
	if (is_stopped(x,y)){
		printf("Node %d",x);
		printf(" is stopped at time %d\n",y);
	}
}

void Trace::test_velocity_has_changed(int x, int y){
	if (velocity_has_changed(x,y)){
		printf("Node'velocity %d",x);
		printf(" has changed at time %d\n",y);
	}
}

void Trace::verificaTrace(data **t){

	for (int x = 0; x < this->node_num; x++){
		for (int y = 0; y < this->time_slot; y++){
			if(t[x][y].x <= -1 || t[x][y].y <= -1 || t[x][y].x >= (this->SCENARIO_WIDTH+1) || t[x][y].y >= (this->SCENARIO_LENGTH +1)){
				std::cout << "trace[" << x << "][" << y << "]: (VT)point out of the scenario: " << "("<< trace[x][y].x << "," << trace[x][y].y << ")" <<'\n';
				exit(0);
			}
		}
	}
	std::cout << " vericou: ok!" << '\n';
}
void Trace::writeResults(const char *file_name){
	std::ofstream results;

	results.open(file_name, std::ofstream::out | std::ofstream::app);
	results << this->metrics[0] << " ATL" << '\n';
	results << this->metrics[1] << " SVC" << '\n';
	results << this->metrics[2] << " SAR" << '\n';
	results << this->metrics[3] << " AVC" << '\n';
	results << this->metrics[4] << " IDSD" << '\n';
	results << this->metrics[5] << " IDTD" << '\n';
	results << this->metrics[6] << " DSA" << '\n';
	results << this->metrics[7] << " DNP" << '\n';
	results << this->metrics[8] << " DNSD" << '\n';
	results << this->metrics[9] << " DLC" << '\n';
	results << this->metrics[10] << " DSA" << '\n';
	results.close();
}
/* ------- struct data management functions ------- */

data ** twoD_array_allocData(int x, int y){  /*--- to allocate bi-dimensional array of data struct ---*/
	data ** matrix;

	matrix = new struct data* [x];
	for(int i=0;i<x;i++)
		matrix[i]=new struct data[y];

	return matrix;
}

data ** reset_trace(data **t, int node_num, int time_slot){ /*--- reset the matrix (trace) for reuse ---*/

	for (int i = 0; i < node_num; i++) {
		for (int j = 0; j < time_slot; j++) {
			t[i][j].start = 0.0;
			t[i][j].x = 0.0;
			t[i][j].y = 0.0;
			t[i][j].next_x = 0.0;
			t[i][j].next_y = 0.0;
			t[i][j].angle = 0.0;
			t[i][j].speed = 0.0;
			t[i][j].timetodestiny = 0;
		}
	}

	return t;
}
