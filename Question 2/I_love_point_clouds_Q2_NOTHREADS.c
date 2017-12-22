#define _GNU_SOURCE
#include <stdio.h>
#include <time.h>
#include <string.h> // helps strcat
#include <stdlib.h> // conv char to double
#include <math.h> // use pow and sqrt
#include <unistd.h>
#define MAX_PTC 20000
//point_cloud1
char read_file_name[]="point_cloud3.txt";   // File to read
char write_file_name[]="cloudpoint3_exit.txt"; // File to write

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

void overwrite_struct(struct PointCloud *ptc,int line_line_num);
void math(struct PointCloud *temp); // Pergunta 1
void delete_x_neg(struct PointCloud *temp); // Pergunta 2.1 e 2.2
void keep_ground(struct PointCloud *ptc); // Pergunta 2.3
void drivable(struct PointCloud *ptc); // Pergunta 3

int main(void){
	int i;
	struct PointCloud pointcloud;
	pointcloud.line_num=0;
	pointcloud.deleted_points=0;
	pointcloud.Stdx=0;
    pointcloud.Stdy=0;
    pointcloud.Stdz=0;

	task1(&pointcloud); // Lê.txt calc Min Max Med Std
	task2(&pointcloud); // nuke x<0
	

    //======== SORT FOR X CODE ========//
    struct str_sort sorted_array[pointcloud.line_num]; //Prof Sort Code
    printf("\n SORT FOR X");
    for(i=0;i<pointcloud.line_num;i++){ // Run all lines
        sorted_array[i].var=pointcloud.x[i]; // Copy to array.var (x is sorted var)
		sorted_array[i].vary=pointcloud.y[i]; // Copy to array.vary (y is sorted var)
		sorted_array[i].varz=pointcloud.z[i]; // Copy to array.varz (z is sorted var)
        sorted_array[i].index=i; // Set array index
        //printf("\n Point | x=%f y=%f z=%f",pointcloud.x[i],pointcloud.y[i],pointcloud.z[i]); // Display PTC
    }
    printf("\n----------------------------------");
    qsort(sorted_array, pointcloud.line_num, sizeof(sorted_array[0]), cmp); // Sort Array <3ms
    for (i=0; i<pointcloud.line_num; i++){
        // Save all sorted points to PointCloud
        pointcloud.x[i] = sorted_array[i].var;
        pointcloud.y[i] = sorted_array[i].vary;
        pointcloud.z[i] = sorted_array[i].varz;
        //printf("\n Point | x=%f y=%f z=%f",pointcloud.x[i],pointcloud.y[i],pointcloud.z[i]);
    }
    //======== SORT FOR X CODE END ========//
	task3(&pointcloud);
    /*
    //======== DEBUG -> DISPLAY SORTED ARRAY ========//
    for(i=0;i<line_num;i++){ // DEBUG -> DISPLAY SORTED ARRAY
        printf("\n Array | x=%f index=%d",sorted_array[i].var,sorted_array[i].index); // Display Sorted Array
    }
    printf("\n----------------------------------");
    */

	/*******************************
	*		POINT PROCESSING
	********************************/

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
    
/*
	for(i=0;i<pointcloud.line_num;i++){ // Print Current PointCloud
		printf("\n Point | x=%f y=%f z=%f",pointcloud.x[i],pointcloud.y[i],pointcloud.z[i]);
	}
	
    for(i=0;i<pointcloud.line_num;i++){ // Print Temporary PTC
        printf("\n Temp Ptc | x=%f y=%f z=%f",temp_ptc[i].x,temp_ptc[i].y,temp_ptc[i].z); 
    }
*/	
    printf("\n Deleted points=%d",pointcloud.deleted_points);
    printf("\n %s has %d points\n",write_file_name,pointcloud.line_num-pointcloud.deleted_points);
    return(0);
}

void* task1(struct PointCloud *ptc) {
	ptc->line_num=0;
	ptc->deleted_points=0;
    int i=0; // For's aux

	//======== READ .TXT FILE ========//
    FILE *ficheiro1;
    //ficheiro1 = (FILE *)malloc(sizeof(FILE)); // SEGMENTATION FAULT
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
    //free(ficheiro1); //Limpa memoria
    //======== READ .TXT FILE END ========

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
}

void* task2(struct PointCloud *ptc){
	delete_x_neg(ptc); // Delete all x<0
	keep_ground(ptc); // Keep ground
}

void* task3(struct PointCloud *ptc){


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
	for(i=0;i<ptc->line_num;i++){ // Corre todas as linhas
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
			ptc->x[i] = 0; // pointcloud.x[i];
            ptc->y[i] = 0; // pointcloud.y[i];
            ptc->z[i] = 0; // pointcloud.z[i];
        }
    }
}


