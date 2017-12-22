#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h> // use pow and sqrt
#include <cstdlib>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <semaphore.h> // Mutex Lib
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/foreach.hpp>

#define MAX_PTC 20000

sem_t mutex; // Inicia Mutex-Semaforo
ros::Publisher newPointCloud;
bool runflag=false;
sensor_msgs::PointCloud2::ConstPtr pointcloud;

struct PointCloud{
    float x[MAX_PTC];
    float y[MAX_PTC];
    float z[MAX_PTC];
	float Minx,Miny,Minz; // Min
	float Maxx,Maxy,Maxz; // Max
	float Mux,Muy,Muz; // Average
	float Stdx,Stdy,Stdz;// Standard deviation
	int line_num;
	int deleted_points;
};

void f1(struct PointCloud *ptc);
void f2(struct PointCloud *ptc);
void f3(sensor_msgs::PointCloud2::ConstPtr pc,struct PointCloud *ptc); //
void handlePointCloud(sensor_msgs::PointCloud2::ConstPtr scan_out);

//==== C Functs ====//
void* task1(struct PointCloud *ptc);
void* task2(struct PointCloud *ptc);
void* task3(struct PointCloud *ptc);

void overwrite_struct(struct PointCloud *temp,int line_line_num);
void math(struct PointCloud *temp); // Pergunta 1
//=================//

void importer(sensor_msgs::PointCloud2::ConstPtr pc,struct PointCloud *ptc); // Imports PointCloud2 to stuct

int main(int argc, char **argv){
	//==== Startup ====//
	struct PointCloud pointcloud_form;
	pointcloud_form.line_num=0;
	pointcloud_form.deleted_points=0;
	pointcloud_form.Stdx=0;
    pointcloud_form.Stdy=0;
    pointcloud_form.Stdz=0;
    
    mlockall(MCL_CURRENT|MCL_FUTURE); // Lock Swapping
	cpu_set_t mask; // Defines CPU(0)
	CPU_ZERO(&mask);
	CPU_SET(0,&mask);
	// CPU Sanity check
	int numCPU = CPU_COUNT(&mask);
	if (numCPU > 1){
		printf("ERROR: CPU number = %i \n",numCPU );
	}
	sched_setaffinity(getpid(),sizeof(cpu_set_t),&mask); // Set CPU affinity mask
	pthread_t thr[3]; // Set fuction thread
	pthread_attr_t attr[3]; // Inicialize thread atribute
	struct sched_param param[3]; // Set Scheduling priority
	int i;
	int error; // Error var
	for(i=0;i<3;i++){ // Thread Generator 0 1 2
		pthread_attr_init(&attr[i]); // 1-Starts default thread
		error=pthread_attr_setschedpolicy(&attr[i], SCHED_FIFO); // 2-Defines scheduling (SCHED_FIFO, SCHED_RR e SCHED_OTHER)
		if (error!=0){
			printf("ERROR: Bad Scheduling Policy\n");
		}
		memset(&(param[i]), 0, sizeof(struct sched_param));
		// Finds top priority and stores it
		param[i].sched_priority = sched_get_priority_max (SCHED_FIFO);
		if(param[i].sched_priority<0){
			printf("ERROR: Getting Priority\n");
		}
		// Set max priority thread
		error=pthread_attr_setschedparam(&attr[i], &param[i]);
		if(error!=0){
			printf("ERROR: Setting Priority \n");
		}
		// Set inherit schedule thread (INHERIT ou EXPLICIT)
		error=pthread_attr_setinheritsched(&attr[i], PTHREAD_EXPLICIT_SCHED);
		if(error!=0){
			printf("ERROR: Setting Inherit Scheduler \n");
		}
	}
	//==== THREADS? ====//
	
	//==================//
	
	//==== Prof Code ====//
	ros::init(argc, argv, "strdemo");
	ros::NodeHandle nh("~");
	newPointCloud = nh.advertise<sensor_msgs::PointCloud>("/output_results", 100);
	ros::Subscriber PointCloudHandlervelodyne = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,handlePointCloud);
	ros::Rate rate(100.0);
	while (nh.ok()){
		if(runflag){
			importer(pointcloud,&pointcloud_form);
			sem_init(&mutex, 0, 1);
			// Start new OS thread
			//printf("\nStarting_threads:\n");
			//printf(" Thread 1: ");
			
			//printf(" Thread 2: ");
			//pthread_create(&thr[1], &attr[1], f2(&pointcloud_form), NULL);
			//printf(" Thread 3: ");
			//pthread_create(&thr[2], &attr[2], f3(pointcloud,&pointcloud_form), NULL);
			//=================//
			
			f1(&pointcloud_form);
			f2(&pointcloud_form);
			f3(pointcloud,&pointcloud_form);
			runflag=false;
		}
	ros::spinOnce();
	rate.sleep();
	}
	//===================//
	return 1;
}

void f1(struct PointCloud *ptc){
	//Part I 1)
	printf("    Starting Task1\n");
	sem_wait(&mutex); // Lock Semaphore
	math(ptc);
	sem_post(&mutex); // Unlock Semaphore
}

void f2(struct PointCloud *ptc){
	printf("    Starting Task2\n");
    sem_wait(&mutex); // Lock Semaphore 

	//==== DELETE X<0 & Keep Ground====// (alinea 2.1|2.2|2.3
	double thr=(ptc->Muz)/1.7;
	for(int i=0;i<ptc->line_num;i++){
        if(ptc->x[i]<0){ // Alinea 2.1 e 2.2 (as 2 no mesmo?) GG
            overwrite_struct(ptc,i);
            ptc->deleted_points++;// Keeps track of deleted points
        }else if((ptc->z[i]>(ptc->Minz+thr))&&(ptc->x[i]!=0)){ // Zmin+Media/2
            overwrite_struct(ptc,i);
            ptc->deleted_points++;
        }
    }
    //================================//

    sem_post(&mutex); // Unlock Semaphore 
}

void f3(sensor_msgs::PointCloud2::ConstPtr pc,struct PointCloud *ptc){
	printf("    Starting Task3\n");
    sem_wait(&mutex); // Lock Semaphore
 
	pointcloud=pc;
	sensor_msgs::PointCloud output;
	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud,output); // Conv 2 para 1 (Para ler pontos)

	//==== DRIVABLE ====//
	
	//==================//


	//==== Exporter ====//
	printf("\n LineNumOut %d",ptc->line_num);
	for(int i=0;i<ptc->line_num;i++){
		output.points[i].x=ptc->x[i];
		output.points[i].y=ptc->y[i];
		output.points[i].z=ptc->z[i];
		//printf("%f %f %f\n",ptc->x[i],ptc->y[i],ptc->z[i]);
	}
	//=================//
	
	output.header=pointcloud->header;
	newPointCloud.publish(output);
	sem_post(&mutex); // Unlock Semaphore
}

//=== Prof ROS Funct ===//
void handlePointCloud(sensor_msgs::PointCloud2::ConstPtr scan_out){
	pointcloud=scan_out;
	sensor_msgs::PointCloud output;
	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud,output);
	std::cout<<"Points: "<<scan_out->height*scan_out->width<<std::endl;
	std::cout<<"Points: "<<output.points.size()<<std::endl;
	//std::cout<<"x="<<output.points[0].x<<"y="<<output.points[0].y<<"z="<<output.points[0].z;
	runflag=true;
}


void importer(sensor_msgs::PointCloud2::ConstPtr pc,struct PointCloud *ptc){
	pointcloud=pc;
	sensor_msgs::PointCloud input; // Inicia Cloud
	sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud,input); // Conv 2 para 1 (Para ler pontos)
	for(int i=0;i<input.points.size();i++){
		ptc->x[i] = input.points[i].x;//pointcloud[i].x; 
		ptc->y[i] = input.points[i].y;//pointcloud[i].y;
        ptc->z[i] = input.points[i].z;//pointcloud[i].z;
        //printf("%f %f %f\n",ptc->x[i],ptc->y[i],ptc->z[i]);
	}
	ptc->line_num=input.points.size();
}

void math(struct PointCloud *ptc) {
	float Sx=0.0;
	float Sy=0.0;
	float Sz=0.0;
    
    ptc->Minx=ptc->x[0];
    ptc->Miny=ptc->y[0];
    ptc->Minz=ptc->z[0];

    ptc->Maxx=ptc->x[0];
    ptc->Maxy=ptc->y[0];
	ptc->Maxz=ptc->z[0];
	int i;
	
    for (i = 0; i < ptc->line_num; i++){
        Sx = Sx + ptc->x[i];
        Sy = Sy + ptc->y[i];
        Sz = Sz + ptc->z[i];
        if(ptc->Minx>ptc->x[i])ptc->Minx=ptc->x[i];
        if(ptc->Miny>ptc->y[i])ptc->Miny=ptc->y[i];
        if(ptc->Minz>ptc->z[i])ptc->Minz=ptc->z[i];

        if(ptc->Maxx<ptc->x[i])ptc->Maxx=ptc->x[i];
        if(ptc->Maxy<ptc->y[i])ptc->Maxy=ptc->y[i];
        if(ptc->Maxz<ptc->z[i])ptc->Maxz=ptc->z[i];
    }

    ptc->Mux = Sx / ptc->line_num; // Average x
    ptc->Muy = Sy / ptc->line_num; // Average y
    ptc->Muz = Sz / ptc->line_num; // Average z

    Sx = 0.0;
    Sy = 0.0;
    Sz = 0.0;
    float Varx=0.0;
    float Vary=0.0;
    float Varz=0.0;
    
    for (i = 0; i < ptc->line_num; i++){
        Sx= Sx + pow((ptc->x[i] - ptc->Mux), 2);
        Sy= Sy + pow((ptc->y[i] - ptc->Muy), 2);
        Sz= Sz + pow((ptc->z[i] - ptc->Muz), 2);
    }
    Varx = Sx / (ptc->line_num-1); // Variance x
    Vary = Sy / (ptc->line_num-1); // Variance y
    Varz = Sz / (ptc->line_num-1); // Variance z

    ptc->Stdx = sqrt(Varx); // Standard deviation x
    ptc->Stdy = sqrt(Vary); // Standard deviation y
    ptc->Stdz = sqrt(Varz); // Standard deviation z
}

void overwrite_struct(struct PointCloud *ptc,int line_line_num){
    // Overwrite values with zero... (zero or null)?? whats better
    ptc->x[line_line_num]=0;
    ptc->y[line_line_num]=0;
    ptc->z[line_line_num]=0;
}
