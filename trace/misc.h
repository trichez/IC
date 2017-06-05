/*
 * misc.h
 * License: GPL 2.0
 *  Created on: 04/02/2010
 *      Author: elmano
 *      elmano.cavalcanti@garanhuns.ifpe.edu.br
 *  Modified: 2017/01-05
 *      Last modified: Matheus Henrique Trichez - UFFS
 *      email: mh.trichez@gmail.com
 */
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <string.h>
#include <ctype.h>

#ifndef MISC_H_
#define MISC_H_


#endif /* MISC_H_ */

// -------- decision tree's structures and functions ------

typedef struct _node{
  int nodeId;
  int nMetric;
  double metricV;
  char * modelName; /*MODEL NAME OR METRIC NAME*/

  struct _node *childL;
  struct _node *childR;
}node;
/*note: this tree keeps de smallers values in the right son */
/*para DECISÃO: peculiaridade desta árvore: os valores menores ficam à esquerda e os maiores à direita*/
struct tree{
  node *root;
};


node *newNode(int id, int metric, double parameter, const char * mName){

    node *n = (node *)malloc(sizeof(node));

    n->nodeId = id;
    n->nMetric = metric;
    n->metricV = parameter;
    n->childL = NULL;
    n->childR = NULL;
    if(mName != NULL){
      n->modelName = (char *)malloc((strlen(mName)+1)*sizeof(char));
      strcpy(n->modelName, mName);
    }else
      n->modelName = NULL;

    return n;
}

node *insertOnTree(node *root, node *n){

  if(root == NULL)
    return n;
  else{
    node *p = root;

    if(p->nodeId < n->nodeId){
      if(p->childL == NULL)
        p->childL = n;
      else
        insertOnTree(p->childL,n);
    }else{
      if(p->childR == NULL)
        p->childR = n;
      else
        insertOnTree(p->childR,n);
    }
  }

  return root;
}
tree * decisionTreeInicializer(tree *t){

  t = (tree *)malloc(sizeof(tree));
  t->root = NULL;
  FILE *f = NULL;
  char str[1024];
  char name[512];
  char word[512];
  double metricValue;
  int id, metricNumber;
  int numberOfnodes = 0;
  int field = 0;
  int i = 0;
  int j = 0;
  int count;


  f = fopen("treeconf.dat", "r");
  if(f){
      if(fgets(str , 1024 , f))
          numberOfnodes = atoi(str);
      else
          exit(0);
      while(fgets(str , 1024 , f) != NULL){
          if(isdigit(str[0]) == 0)
              continue;
          i = 0;
          field = 0;
          do{
              if(str[i] != ' ' && str[i] != '\n'){
                  word[j] = str[i];
                  j++;
              }else{
                  word[j] = '\0';
                  j = 0;
                  field++;
                  switch(field){
                      case 1:
                          id = atoi(word);
                          break;
                      case 2:
                          metricNumber = atoi(word);
                          break;
                      case 3:
                          metricValue = atof(word);
                          break;
                      case 4:
                          strcpy(name, word);
                          break;
                  }
              }
              i++;
          }while(str[i] != '\0');
          for(count = 0; count < numberOfnodes; count++) {
              t->root = insertOnTree(t->root, newNode(id, metricNumber, metricValue, name));
          }
      };
  }else{
      puts("couldn't find file treeconf.dat");
      exit(0);
  }

  return t;
}


char  *modelIdentifier(node *r, double *m){
  if(r == NULL)
    exit(0);
  char *str = NULL;
  node *aux = r;

  if(aux->metricV <= m[aux->nMetric]){//   vare pra direita
    if(aux->childR != NULL)
      str = modelIdentifier(aux->childR, m);
    else
      str = aux->modelName;
  }else{                               //   vare pra esquerda que ...
    if(aux->childL != NULL)
      str = modelIdentifier(aux->childL, m);
    else
      str =  aux->modelName;
  }
  return str;
}

// -------------------------------------

char * formatParams(const char *s){
  int slen = strlen(s);
  char * str = (char *)malloc(slen * sizeof(char) + 1);

  for (int i= 0; i < slen ; i++) {
    if(s[i] == ' ')
      str[i] = '_';
    else if(s[i] == '-')
      str[i] = '_';
    else
      str[i]=s[i];
  }
  str[slen] = '\0';

  return str;
}



#define PI 3.1415926
static const char alph[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
//----------

char * genRandomS(int l){ /*generates a pseudo-random word with l-1 chars. in C format(with null terminator) */
  char * str = new char[l];

  for(int i = 0; i < (l - 1); i++)
    str[i] = alph[rand() % (strlen(alph)- 1)];
   str[l-1] = '\0';

  return str;
}

int factorial(int n){
   int x = 1;

   for (int i = 1; i <= n ; i++)
      x *= i;

  return x;
}
//----------

//-----------------------------------------------------------------//
//Modular function
int my_mod ( int i, int j){

  return int(double(i)/(double)j);
}

//-----------------------------------------------------------------//
//--------Calculate the distance between two nodes-----------------//
double dist(double a0, double b0, double a1, double b1){
	double distx,disty;
	distx=a0-a1;
	disty=b0-b1;


	return sqrt(distx*distx+disty*disty);
}

//-----------------------------------------------------------------//
//--------Calculate the angle between two nodes--------------------//
double get_angle(double a0,double b0,double a1,double b1){
	double distx,disty;
	double angle = 0;
	distx = a1-a0;
	disty = b1-b0;

	if(disty==0 && distx==0)
		angle = 0 ;
	else if(distx==0 && disty>0)
		angle = PI/2;
	else if(distx==0 && disty<0)
		angle =-PI/2;
	else if (disty>=0 && distx>0)   //1
		angle=atan((disty)/(distx));
	else if (disty<=0 && distx>0)  //4
		angle=atan((disty)/(distx));
	else if (disty>=0 && distx<0)   //2
		angle=atan((disty)/(distx))+PI;
	else if (disty<=0 && distx<0)    //3
		angle=atan((disty)/(distx))-PI;
	angle = floor(angle * 100 +  0.5) / 100;
	return angle; //to avoid returning values like 1.72E-12
}


//-----------------------------------------------------------------//
//-----------------------------------------------------------------//
//-----------------------------------------------------------------//
//-----------------------------------------------------------------//
//return the max number
double max(double a, double b){
	if(a>=b)
		return a;
	else
		return b;
}


//-----------------------------------------------------------------//
//return the min number
double min(double a, double b){
	if(a<=b)
		return a;
	else
		return b;
}


//-----------------------------------------------------------------//
//-----------------------------------------------------------------//
//-----------------------------------------------------------------//
//-----------------------------------------------------------------//
//----------- Calculate the Mobility metric ------------------------//
double relative_speed(double speed1,double angle1,double speed2, double angle2) {// Velocidade Relativa (VR)

	double rv;

	rv=sqrt(speed1*speed1+speed2*speed2-2*speed1*speed2*cos(angle1-angle2));

	return rv;
}


//-----------------------------------------------------------------//
//----------- Calculate the correlation metric ---------------------//
double DSDijt(double speed1, double angle1, double speed2, double angle2){
	double x_speed1 = speed1*cos(angle1);
	double y_speed1 = speed1*sin(angle1);

	double x_speed2 = speed2*cos(angle2);
	double y_speed2 = speed2*sin(angle2);

	double cor = 0.0;

	if( (speed1*speed2!=0.0) && (max(speed1,speed2)!=0) ){
		cor = (x_speed1*x_speed2 + y_speed1*y_speed2)/(speed1*speed2)*(min(speed1,speed2)/max(speed1,speed2));
		return cor;
	}
	else
		return 0.0;
}

double basic_correlation_positive(double speed1, double angle1, double speed2, double angle2){
	double x_speed1 = speed1*cos(angle1);
	double y_speed1 = speed1*sin(angle1);

	double x_speed2 = speed2*cos(angle2);
	double y_speed2 = speed2*sin(angle2);

	double cor = 0.0;

	if( (speed1*speed2!=0.0) && (max(speed1,speed2)!=0) ){
		cor = (x_speed1*x_speed2 + y_speed1*y_speed2)/(speed1*speed2)*(min(speed1,speed2)/max(speed1,speed2));
		return cor <= 0.0 ? 0.0 : cor; //negative cor does not make sense for temporal dependence
	}
	else
		return 0.0;
}


//-----------------------------------------------------------------//
//----------- Calculate the angle correlation
double angle_correlation(double speed1, double angle1, double speed2, double angle2){
	double x_speed1 = speed1*cos(angle1);
	double y_speed1 = speed1*sin(angle1);
	double x_speed2 = speed2*cos(angle2);
	double y_speed2 = speed2*sin(angle2);

	if(speed1*speed2!=0.0)
		return (x_speed1*x_speed2 + y_speed1*y_speed2)/(speed1*speed2);
	else
		return 0.0;
}


//-----------------------------------------------------------------//
//------------ Calculate the speed correlation
double speed_correlation(double speed1, double angle1,double speed2,double angle2){
	if(max(speed1,speed2)!=0.0)
		return (min(speed1,speed2)/max(speed1,speed2));
	else
		return 0.0;
}

//-----------------------------------------------------------------//
//-----------------------------------------------------------------//

double getStd(double array[], double mean, int size){

	double desvio = 0;
	double sum = 0;

	for (int k = 0; k < size; ++k) {
		desvio = array[k] - mean;
		desvio *= desvio;
		sum += desvio;
	}

	return sqrt(sum/size);
}

double getAverage(double array[], int size){

	double sum = 0;
	for (int k = 0; k < size; ++k) {
		sum += array[k];
	}

	return sum/size;
}

double getAverageNotZero(double array[], int size){

	//discard null values (0).
	double sum = 0;
	int count = 0;

	for (int k = 0; k < size && array[k] > 0; ++k) {
		sum += array[k];
		count++;
	}

	return sum/(double)count;
}

double getStdNotZero(double array[], double mean, int size){

	double desvio = 0;
	double sum = 0;
	int count = 0;

	for (int k = 0; k < size && array[k] > 0; ++k){
		desvio = array[k] - mean;
		desvio *= desvio;
		sum += desvio;
		count++;
	}

	return sqrt(sum/(double)count);
}

double coefficient_of_variation(double mean, double std){
	return std/mean;
}
