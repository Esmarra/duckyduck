#define _GNU_SOURCE
#include <stdio.h>
#include <time.h>
#include <string.h> // helps strcat
#include <stdlib.h> // conv char to double
#include <math.h> // use pow and sqrt
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <semaphore.h> // Mutex Lib
#define MAX_PTC 20000

sem_t mutex; // Inicia Mutex-Semaforo
//point_cloud1
char read_file_name[]="point_cloud1.txt";   // File to read
char write_file_name[]="cloudpoint1_exit_qsortx.txt"; // File to write

struct PointCloud{
	double x[MAX_PTC];
	double y[MAX_PTC];
	double z[MAX_PTC];
	float Minx,Miny,Minz; // Min
	float Maxx,Maxy,Maxz; // Max
	float Mux,Muy,Muz; // Average
	float Stdx,Stdy,Stdz; // Standard deviation
	int line_num;
	int deleted_points;
};

typedef struct test{ //DELTE THIS?
    double x;
    double y;
    double z;
};

struct str_sort {
    double var; //x
    double vary; //y
    double varz; //z
    int index;
};

int cmp(const void *a,const void *b) {
    struct str_sort *a1 = (struct str_sort *)a;
    struct str_sort *a2 = (struct str_sort *)b;
    return ((*a1).var >= (*a2).var) ? 1 : -1;
}
void* task1(struct PointCloud *ptc);
void* task2(struct PointCloud *ptc);
void* task3(struct PointCloud *ptc);

void overwrite_struct(struct PointCloud *temp,int line_line_num);
void math(struct PointCloud *temp); // Pergunta 1
void delete_x_neg(struct PointCloud *temp); // Pergunta 2.1 e 2.2
void keep_ground(struct PointCloud *ptc); // Pergunta 2.3
void drivable(struct PointCloud *ptc); // Pergunta 3



int main(int argc, char * argv []){
	read_file_name[0]='\0'; // Clear char
	strcat(read_file_name,argv[1]);// Append console argv to read_file_name
	struct PointCloud pointcloud;
	pointcloud.line_num=0;
	pointcloud.deleted_points=0;
	pointcloud.Stdx=0;
    pointcloud.Stdy=0;
    pointcloud.Stdz=0;

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
	sem_init(&mutex, 0, 1);
	// Start new OS thread
	printf("\nStarting_threads:\n");
	printf(" Thread 1: ");
	pthread_create(&thr[0], &attr[0], task1(&pointcloud), NULL);
	printf(" Thread 2: ");
	pthread_create(&thr[1], &attr[1], task2(&pointcloud), NULL);
	printf(" Thread 3: ");
	pthread_create(&thr[2], &attr[2], task3(&pointcloud), NULL);
	
	
	pthread_join(thr[0],NULL);
	pthread_join(thr[1],NULL);
	pthread_join(thr[2],NULL);
	
    sem_destroy(&mutex); // destroy mutex before ending the program
    return(0);
	
/*
    // Write New PCL
    struct test temp_ptc[pointcloud.line_num-pointcloud.deleted_points];
    int n_ptc=0;
    FILE *outfile;
    outfile = fopen(write_file_name,"w");
    for(i=0;i<pointcloud.line_num;i++){
        if((pointcloud.x[i])!=0){
            fprintf(outfile,"%f %f %f\n",pointcloud.x[i],pointcloud.y[i],pointcloud.z[i]);
            // Store to other struct
            temp_ptc[n_ptc].x=pointcloud.x[i];
            temp_ptc[n_ptc].y=pointcloud.y[i];
            temp_ptc[n_ptc].z=pointcloud.z[i];
            n_ptc++;
        }
    }
    fclose(outfile);
    //=== Write New PCL END ===
*/
	//
	// printf("\n Deleted points=%d",pointcloud.deleted_points);
	//return(0);
}

void* task1(struct PointCloud *ptc) { // Le e faz math
	printf("    Starting Task1\n");
    sem_wait(&mutex); // Lock Semaphore
    ptc->line_num=0;
    ptc->deleted_points=0;
    int i=0; // For's aux

    //======== READ .TXT FILE ========//
    FILE *ficheiro1;
    ficheiro1 = (FILE *)malloc(sizeof(FILE));
    ficheiro1 = fopen(read_file_name,"rt"); // Inicializa ficheiro de leitura
	while( !feof (ficheiro1) ){
        float x,y,z=0.0;
        fscanf(ficheiro1, "%f %f %f", &x, &y, &z);
        ptc->x[ptc->line_num] = x;
        ptc->y[ptc->line_num] = y;
        ptc->z[ptc->line_num] = z;
        ptc->line_num++;
    }
    ptc->line_num-=1;
    fclose(ficheiro1);// Close file
    //free(ficheiro1); //Limpa memoria <-- Causa Segmentation fault
    //======== READ .TXT FILE END ========//

    //======== MATH ========//
    math(ptc);
    //======== MATH END ========//
	printf("\n----------------------------------\n");
	printf(" File \"%s\" has %d lines\n",read_file_name,ptc->line_num);
    printf(" Min | X:%f Y:%f Z=%f\n",ptc->Minx,ptc->Miny,ptc->Minz);
    printf(" Max | X:%f Y:%f Z=%f\n",ptc->Maxx,ptc->Maxy,ptc->Maxz);
    printf(" Average | X:%f Y:%f Z=%f\n",ptc->Mux,ptc->Muy,ptc->Muz);
    printf(" Standard deviation | X:%f Y:%f Z=%f",ptc->Stdx,ptc->Stdy,ptc->Stdz);
    printf("\n----------------------------------\n");

    sem_post(&mutex); // Unlock Semaphore 
}

void* task2(struct PointCloud *ptc){ // Delete x<0 ,  Keep ground
	printf("    Starting Task2\n");
    sem_wait(&mutex); // Lock Semaphore 
    delete_x_neg(ptc); // Delete all x<0 e Cluster points
    keep_ground(ptc); // Keep Ground only
    sem_post(&mutex); // Unlock Semaphore 
}

void* task3(struct PointCloud *ptc){ // DRIVABLE
	printf("    Starting Task3\n");
    sem_wait(&mutex); // Lock Semaphore
    
    sem_post(&mutex); // Unlock Semaphore
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
		if( ptc->x[i] !=0 &&  abs(ptc->x[i]>ptc->Stdx*2) || abs(ptc->y[i])>ptc->Stdy-1 || abs(ptc->z[i]>ptc->Stdz) ){			
			ptc->x[i] = 0;//pointcloud[i].x; 
            ptc->y[i] = 0;//pointcloud[i].y;
            ptc->z[i] = 0;//pointcloud[i].z;
        }
    }
}

