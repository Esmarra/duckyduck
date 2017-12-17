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
#define BILLION 1000000000

void* f_timer(void* arg);
//point_cloud1
char read_file_name[]="point_cloud1.txt";   // File to read

char temp; // Aux var to read file
char makes[]=""; // Makes char array(string)

struct tabela{
    double x;
    double y;
    double z;
}pointcloud[20000];

typedef struct test{
    double x;
    double y;
    double z;
};

float Minx,Miny,Minz; // Min
float Maxx,Maxy,Maxz; // Max
float Mux,Muy,Muz; // Average
float Stdx,Stdy,Stdz;// Standard deviation

struct str_sort {
    double var;
    int index;
};

int cmp(const void *a,const void *b) {
    struct str_sort *a1 = (struct str_sort *)a;
    struct str_sort *a2 = (struct str_sort *)b;
    return ((*a1).var >= (*a2).var) ? 1 : -1;
}

void overwrite_neg_x(int line_line_num);

int main(void){
	//strcat(read_file_name,argv[1]);
	//printf("%s",argv[1]);
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
	// Finds top priority and stores it
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

void overwrite_neg_x(int line_line_num){
    // Overwrite values with zero... (zero or null)?? whats better
    pointcloud[line_line_num].x=0;
    pointcloud[line_line_num].y=0;
    pointcloud[line_line_num].z=0;
}

void* f_timer(void* arg){
	struct timespec strT,endT;
	double *performance=malloc(3*sizeof(double)); // Execution time for t1 t2 t3
    int i=0; // For's aux
    int cord=0; // Aux Store in PTC Table (0=x,1=y,2=z)
    int line_num=0; // Num Lines
    // Ler ficheiro
    FILE *ficheiro1;
    ficheiro1 = fopen(read_file_name,"rt"); // Inicializa ficheiro de leitura
    while((temp=fgetc(ficheiro1))!=EOF){ // read's car br car (omg...)
        if(temp==' '){ // looks for space
            //printf("<SPACE>");
            //printf("\n X=%s",makes);
            if(cord==0)pointcloud[line_num].x=strtod(makes,NULL); // conv makes[] to double, stores PTC[i].x
            if(cord==1)pointcloud[line_num].y=strtod(makes,NULL); // conv makes[] to double, stores PTC[i].y
            *makes='\0'; // clear makes[]
            cord=cord+1; // set next coord (0=x,1=y,2=z)

        }else if(temp=='\n'){ // Detects file endl
                //printf("<end string>");
                pointcloud[line_num].z=strtod(makes,NULL); // conv makes[] to double, stores PTC[i].z
                //printf("\n Z=%s",makes);
                *makes='\0'; // clear makes[]
                line_num=line_num+1; // count number of lines
                cord=0; // Reset coord(next coord is 0 aka x)
        }else{
             strcat(makes,&temp); // Adds char to string makes[]
             //printf("%c",temp);
        }
    }
    fclose(ficheiro1); // Close file
    // READS .TXT FILE

    //=== MATH ===
    float Sx,Sy,Sz=0.0;
    Minx=pointcloud[0].x;
    Miny=pointcloud[0].y;
    Minz=pointcloud[0].z;

    Maxx=pointcloud[0].x;
    Maxy=pointcloud[0].y;
    Maxz=pointcloud[0].z;
    for (i = 0; i < line_num; i++){
        Sx = Sx + pointcloud[i].x;
        Sy = Sy + pointcloud[i].y;
        Sz = Sz + pointcloud[i].z;
        if(Minx>pointcloud[i].x)Minx=pointcloud[i].x;
        if(Miny>pointcloud[i].y)Miny=pointcloud[i].y;
        if(Minz>pointcloud[i].z)Minz=pointcloud[i].z;

        if(Maxx<pointcloud[i].x)Maxx=pointcloud[i].x;
        if(Maxy<pointcloud[i].y)Maxy=pointcloud[i].y;
        if(Maxz<pointcloud[i].z)Maxz=pointcloud[i].z;
    }
    //=== MATH END===

    int deleted_points_x=0;
    int deleted_points_wallx=0;
    double threshold=0.400; // <----- threshold 2 agressive?

	clock_gettime(CLOCK_MONOTONIC,&strT); // f1 start time
	// F1
    for(i=0;i<line_num;i++){
        if(pointcloud[i].x<0){ // Alinea 2.1 e 2.2 (as 2 no mesmo?) GG
            overwrite_neg_x(i);
            deleted_points_x++; // Keeps track of deleted points
        }
    }
    // F1 END
	clock_gettime(CLOCK_MONOTONIC,&endT); // f1 end time
	performance[0] += (endT.tv_sec-strT.tv_sec)+(endT.tv_nsec-strT.tv_nsec)/(float)BILLION;

	clock_gettime(CLOCK_MONOTONIC,&strT); // f2 start time
	// F2
    for(i=0;i<line_num;i++){
        if ((pointcloud[i].z>0)&&(pointcloud[i].x!=0)){
            overwrite_neg_x(i);
            deleted_points_x++;
        }
    }
    // F2 END
	clock_gettime(CLOCK_MONOTONIC,&endT); // f2 end time
	performance[1] += (endT.tv_sec-strT.tv_sec)+(endT.tv_nsec-strT.tv_nsec)/(float)BILLION;

	clock_gettime(CLOCK_MONOTONIC,&strT); // f3 start time
	// F3
    for(i=0;i<line_num;i++){
        if((pointcloud[i+1].x>(pointcloud[i].x-threshold))&&(pointcloud[i+1].x<(pointcloud[i].x+threshold)&&(pointcloud[i].x!=0))){ // Keep only ground/road
            overwrite_neg_x(i); // A for effort :D
            deleted_points_wallx++;
        }
    }
    // F3 END
	clock_gettime(CLOCK_MONOTONIC,&endT); // f3 end time
	performance[2] += (endT.tv_sec-strT.tv_sec)+(endT.tv_nsec-strT.tv_nsec)/(float)BILLION;
	pthread_exit((void*)performance); // End thread, shows performance
}
