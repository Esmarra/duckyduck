#define _GNU_SOURCE
#include <stdio.h>
#include <time.h>
#include <string.h> // helps strcat
#include <stdlib.h> // conv char to double
#include <math.h> // use pow and sqrt
#include <unistd.h>
#include <stdint.h> // int_8 int_32 etc
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <semaphore.h> // Mutex Lib
#define MAX_PTC 20000

char read_file_name[]="point_cloud1.txt";   // File to read

struct PointCloud{
	float x[MAX_PTC];
	float y[MAX_PTC];
	float z[MAX_PTC];
	float Minx,Miny,Minz; // Min
	float Maxx,Maxy,Maxz; // Max
	float Mux,Muy,Muz; // Average
	float Stdx,Stdy,Stdz; // Standard deviation
	int line_num;
	int deleted_points;
};

void read_file(struct PointCloud *temp, FILE *fptr, int file);
void* task1(struct PointCloud *temp);

int main(){
	struct PointCloud pointcloud;
	pointcloud.Minx=0;
	pointcloud.Miny=0;
	pointcloud.Minz=0; // Min
	pointcloud.Maxx=0;
	pointcloud.Maxy=0;
	pointcloud.Maxz=0; // Max
	pointcloud.Mux=0;
	pointcloud.Muy=0;
	pointcloud.Muz=0;// Average
	pointcloud.Stdx=0;
	pointcloud.Stdy=0;
	pointcloud.Stdz=0; // Standard deviation
	pointcloud.line_num=0;
	
	pointcloud.deleted_points=0;
	task1(&pointcloud);

}

void* task1(struct PointCloud *temp) {
	FILE *readfile;
	readfile = (FILE *)malloc(sizeof(FILE));
	read_file(temp, readfile, 1);
	free(readfile);
}

void read_file(struct PointCloud *temp, FILE *fptr, int file){ // mudar o int file para uma string
    if(file==1){
        fptr = fopen("point_cloud1.txt","r");  // Open the file 1
        printf("\nReading point_cloud1.txt\n\n");
    }
    if(file==2){
        fptr = fopen("point_cloud2.txt","r");  // Open the file 2
        printf("\nReading point_cloud2.txt\n\n");
    }
    if(file==3){
        fptr = fopen("point_cloud3.txt","r");  // Open the file 3
        printf("\nReading point_cloud3.txt\n\n");
    }
    if(fptr == NULL){
        perror("fopen()");
        exit(1);
    }

    // Verify if the document reached to an end
    while( !feof (fptr) ){
        // Saves the values from .txt file to the variables
        float _x = 0.0, _y = 0.0, _z = 0.0;
        fscanf(fptr, "%f %f %f", &_x, &_y, &_z);

        temp->x[temp->line_num] = _x;
        temp->y[temp->line_num] = _y;
        temp->z[temp->line_num] = _z;

        temp->line_num++;  // last line of coord is all 0
    }
    temp->line_num-=1;

    fclose(fptr);
}

