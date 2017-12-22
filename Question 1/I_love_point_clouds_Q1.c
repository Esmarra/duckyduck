#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h> //conv char to double
#include <math.h> // use pow and sqrt
#include <unistd.h>
#define MAX_PTC 20000

char read_file_name[] ="point_cloud1.txt";   // File to read

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

void math(struct PointCloud *temp);

int main(int argc, char * argv []){
	struct PointCloud pointcloud;
	pointcloud.line_num=0;
	pointcloud.deleted_points=0;

	read_file_name[0]='\0'; // Clear char
	strcat(read_file_name,argv[1]);// Append console argv to read_file_name

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

    //======== MATH ========//
	math(&pointcloud);
    //======== MATH END ========//
    
    printf("\n----------------------------------\n");
    printf(" File \"%s\" has %d lines\n",read_file_name,pointcloud.line_num);
    printf(" Min | X:%f Y:%f Z=%f\n",pointcloud.Minx,pointcloud.Miny,pointcloud.Minz);
    printf(" Max | X:%f Y:%f Z=%f\n",pointcloud.Maxx,pointcloud.Maxy,pointcloud.Maxz);
    printf(" Average | X:%f Y:%f Z=%f\n",pointcloud.Mux,pointcloud.Muy,pointcloud.Muz);
    printf(" Standard deviation | X:%f Y:%f Z=%f",pointcloud.Stdx,pointcloud.Stdy,pointcloud.Stdz);
    printf("\n----------------------------------\n");
    return(0);
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
