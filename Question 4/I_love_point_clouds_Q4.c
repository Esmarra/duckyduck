#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <stdlib.h> //conv char to double
#include <math.h> // use pow and sqrt
#include <unistd.h>
#define MAX_PTC 20000
#define BILLION 1000000000

//point_cloud1
char read_file_name[]="point_cloud1.txt";   // File to read

struct PointCloud{
    double x[MAX_PTC];
    double y[MAX_PTC];
    double z[MAX_PTC];
	float Minx,Miny,Minz; // Min
	float Maxx,Maxy,Maxz; // Max
	float Mux,Muy,Muz; // Average
	float Stdx,Stdy,Stdz;// Standard deviation
	int line_num;
	int deleted_points;
};

struct str_sort {
    double var;
    int index;
};

int cmp(const void *a,const void *b) {
    struct str_sort *a1 = (struct str_sort *)a;
    struct str_sort *a2 = (struct str_sort *)b;
    return ((*a1).var >= (*a2).var) ? 1 : -1;
}

void* f_timer(void* arg);

void overwrite_struct(struct PointCloud *temp,int line_line_num);
void math(struct PointCloud *temp); // Pergunta 1
void delete_x_neg(struct PointCloud *temp); // Pergunta 2.1 e 2.2
void keep_ground(struct PointCloud *ptc); // Pergunta 2.3
void drivable(struct PointCloud *ptc); // Pergunta 3

int main(int argc, char * argv []){
	read_file_name[0]='\0'; // Clear char
	strcat(read_file_name,argv[1]);// Append console argv to read_file_name
	
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
	pthread_t thr; // Set fuction thread
	pthread_attr_t attr; // Inicialize thread atribute
	struct sched_param param; // Set Scheduling priority
	void *exe_time; // Execution time for t1 t2 t3
	int error; // Error var
	pthread_attr_init(&attr); // Starts default thread
	error=pthread_attr_setschedpolicy(&attr, SCHED_FIFO); // Defines scheduling (SCHED_FIFO, SCHED_RR e SCHED_OTHER)

	if (error!=0){
		printf("ERROR: Bad Scheduling Policy\n");
	}
	// Finds top priority and stores it0
	param.sched_priority = sched_get_priority_max (SCHED_FIFO);
	if(param.sched_priority<0){
		printf("ERROR: Getting Priority\n");
	}
	// Set max priority thread
	error=pthread_attr_setschedparam(&attr, &param);
	if(error!=0){
		printf("ERROR: Setting Priority \n");
	}
	// Set inherit schedule thread (INHERIT ou EXPLICIT)
	error=pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if(error!=0){
		printf("ERROR: Setting Inherit Scheduler \n");
	}
	// Start new OS thread
	error=pthread_create(&thr, &attr, f_timer, NULL);
	if (error!=0){
		printf("ERROR: Creating Thread \n");
	}
	// Wait for thread exe to end, store exe_time
	pthread_join(thr,&exe_time);
	double *performance=(double*)exe_time;

	printf("Time F1 = %lf \n", performance[0]);
	printf("Time F2 = %lf \n", performance[1]);
	printf("Time F3 = %lf \n", performance[2]);
	pthread_attr_destroy(&attr); // Destroy thread attribute
    return(0);
}


void math(struct PointCloud *ptc) {
	float Sx,Sy,Sz=0.0;
    
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
    float Varx,Vary,Varz;
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

void delete_x_neg(struct PointCloud *ptc){
	int i;
	for(i=0;i<ptc->line_num;i++){
        if(ptc->x[i]<0){ // Alinea 2.1 e 2.2 (as 2 no mesmo?) GG
            overwrite_struct(ptc,i);
            ptc->deleted_points++;// Keeps track of deleted points
        }
    }
}

void keep_ground(struct PointCloud *ptc){
	int i;
    double thr=(ptc->Muz)/2; // <----- threshold 2 agressive? Media/2
    for(i=0;i<ptc->line_num;i++){
        if ((ptc->z[i]>(ptc->Minz+thr))&&(ptc->x[i]!=0)){ // Zmin+Media/2
            overwrite_struct(ptc,i);
            ptc->deleted_points++;
        }
    }
}

void drivable(struct PointCloud *ptc){ 
	int i;
	for( i=0; i<ptc->line_num; i++){ // Pessoa mambo jambo
		if( (ptc->x[i] !=0) && (abs(ptc->x[i]>ptc->Stdx*2))||(abs(ptc->y[i])>ptc->Stdy-1) || (abs(ptc->z[i]>ptc->Stdz)) ){			
			ptc->x[i] = 0;//ointcloud[i].x; 
            ptc->y[i] = 0;//pointcloud[i].y;
            ptc->z[i] = 0;//pointcloud[i].z;
        }
    }
}

void* f_timer(void* arg){
	struct PointCloud pointcloud;
	pointcloud.line_num=0;
	pointcloud.deleted_points=0;
	printf("Timing functions with: %s\n",read_file_name);
	
	struct timespec strT,endT;
	double *performance=malloc(3*sizeof(double)); // Execution time for t1 t2 t3
	
	//======== READ .TXT FILE ========//
	FILE *ficheiro1;
	//ficheiro1 = (FILE *)malloc(sizeof(FILE)); // SEGMENTATION FAULT
	ficheiro1 = fopen(read_file_name,"rt"); // Inicializa ficheiro de leitura
	while( !feof (ficheiro1) ){
		float x,y,z=0.0;
		fscanf(ficheiro1, "%f %f %f", &x, &y, &z);
		pointcloud.x[pointcloud.line_num] = x;
		pointcloud.y[pointcloud.line_num] = y;
		pointcloud.z[pointcloud.line_num] = z;
		pointcloud.line_num++;
    }
	pointcloud.line_num-=1;
	fclose(ficheiro1);// Close file
	//free(ficheiro1); //Limpa memoria
	//======== READ .TXT FILE END ========

	clock_gettime(CLOCK_MONOTONIC,&strT); // f1 start time
	// F1 --> math
	math(&pointcloud);
	// F1 END
	clock_gettime(CLOCK_MONOTONIC,&endT); // f1 end time
	performance[0] += (endT.tv_sec-strT.tv_sec)+(endT.tv_nsec-strT.tv_nsec)/(float)BILLION;

	
	clock_gettime(CLOCK_MONOTONIC,&strT); // f2 start time
	// F2 --> data processing (x<0|z<0)
	delete_x_neg(&pointcloud);
	keep_ground(&pointcloud);
	// F2 END
	clock_gettime(CLOCK_MONOTONIC,&endT); // f2 end time
	performance[1] += (endT.tv_sec-strT.tv_sec)+(endT.tv_nsec-strT.tv_nsec)/(float)BILLION;

	
	clock_gettime(CLOCK_MONOTONIC,&strT); // f3 start time
	// F3 --> Drivable Points
	drivable(&pointcloud);
	// F3 END
	clock_gettime(CLOCK_MONOTONIC,&endT); // f3 end time
	performance[2] += (endT.tv_sec-strT.tv_sec)+(endT.tv_nsec-strT.tv_nsec)/(float)BILLION;
	pthread_exit((void*)performance); // End thread, shows performance
}
