/***********************************************************
    jacobi.c -- Jacobi (ヤコビ) 法
***********************************************************/
#include <math.h>
#include <iostream>
#include "set_define.h"
#include "struct.h"
#include "mymath.h"
#include "list_handle.h"
#include "matutil.h"  /* 行列用小道具集 */
#include "pm6.h"  /* 行列用小道具集 */

#define EPS         1E-6  /* 許容誤差 */
#define TINY        1E-20 /* 0 と見なしてよい値 */
#define MAX_ITER    100   /* 最大の繰返し数 */
#define forall(i)   for (i = 0; i < n; i++)
#define rotate(a, i, j, k, l) {      \
    double x = a[i][j], y = a[k][l]; \
    a[i][j] = x * c - y * s;         \
    a[k][l] = x * s + y * c;         }

int jacobi(int n, matrix a, matrix w)
{
    int i, j, k, iter;
    double t, c, s, tolerance, offdiag;
    vector v;

    s = offdiag = 0;
    forall(j) {
        forall(k) w[j][k] = 0;
        w[j][j] = 1;  s += a[j][j] * a[j][j];
        for (k = j + 1; k < n; k++)
            offdiag += a[j][k] * a[j][k];
    }
    tolerance = EPS * EPS * (s / 2 + offdiag);
    for (iter = 1; iter <= MAX_ITER; iter++) {
        offdiag = 0;
        for (j = 0; j < n - 1; j++)
            for (k = j + 1; k < n; k++)
                offdiag += a[j][k] * a[j][k];
        #ifdef TEST
            printf("%4d: %g\n",
                iter, sqrt(2 * offdiag / (n * (n - 1))));
        #endif
        if (offdiag < tolerance) break;
        for (j = 0; j < n - 1; j++) {
            for (k = j + 1; k < n; k++) {
                if (fabs(a[j][k]) < TINY) continue;
                t = (a[k][k] - a[j][j]) / (2 * a[j][k]);
                if (t >= 0) t = 1 / (t + sqrt(t * t + 1));
                else        t = 1 / (t - sqrt(t * t + 1));
                c = 1 / sqrt(t * t + 1);
                s = t * c;  t *= a[j][k];
                a[j][j] -= t;  a[k][k] += t;  a[j][k] = 0;
                for (i = 0; i < j; i++)     rotate(a, i, j, i, k)
                for (i = j + 1; i < k; i++) rotate(a, j, i, i, k)
                for (i = k + 1; i < n; i++) rotate(a, j, i, k, i)
                forall(i)                   rotate(w, j, i, k, i)
            }
        }
    }
    if (iter > MAX_ITER) return EXIT_FAILURE;  /* 収束せず */
    for (i = 0; i < n - 1; i++) {
        k = i;  t = a[k][k];
        for (j = i + 1; j < n; j++)
            if (a[j][j] > t) {  k = j;  t = a[k][k];  }
        a[k][k] = a[i][i];  a[i][i] = t;
        v = w[k];  w[k] = w[i];  w[i] = v;
    }
    return EXIT_SUCCESS;  /* 成功 */
}




double quadrics(object_data *o, int cn){
    int i, j, k, n=3;
	int nt,nv=0;
    double p, inv;
    matrix a, b, w, z, sum;
//    vector lambda;
	cluster_data *c;
	triangle_data **tq;
	static int first=TRUE;
	static vertex_data **vq;

	if(first==TRUE)
		vq = (vertex_data **)malloc(sizeof(vertex_data*)*o->num_of_ver);
	first = FALSE;

	c = &o->clusters[cn];
	nt = c->number_of_triangle;
	tq = c->triangles_que;

    a = new_matrix(n, n);
    b = new_matrix(n, n);
    w = new_matrix(n, n);
    sum = new_matrix(n, n);
	z = new_matrix(n, n);
//    lambda = new_vector(n);

	for(i=0;i<n;i++){
		for(j=0;j<n;j++){
			a[i][j] = 0;
			b[i][j] = 0;
			w[i][j] = 0;
			sum[i][j] = 0;
			z[i][j] = 0;
		}
	}
	for(j=0;j<3;j++){
		c->center[j] = 0;
	}

	//クラスタに属する頂点判別
	for(i=0;i<nt;i++){
		for(j=0;j<3;j++){
			if(tq[i]->ver[j]->cluster_mark==cn) continue;
			vq[nv++] = tq[i]->ver[j];
			tq[i]->ver[j]->cluster_mark = cn;
		}
	}
	c->number_of_vertex = nv;
	for(i=0;i<nv;i++){
		for(j=0;j<3;j++){
			c->center[j] += vq[i]->pos[j];
			for(k=0;k<3;k++){
				sum[j][k] += vq[i]->pos[j] * vq[i]->pos[k];
			}
		}
	}
	for(j=0;j<3;j++){
		c->center[j] /= c->number_of_vertex;
	}
	for(j=0;j<3;j++){
		for(k=0;k<3;k++){
			z[j][k] = sum[j][k] - c->center[j] * c->center[k] * c->number_of_vertex;
		}
	}
    if (jacobi(3, z, w) == EXIT_FAILURE) printf("収束しません\n");

	for(i=0;i<3;i++){
		c->normal[i] = w[2][i];
		for(j=0;j<3;j++){
			c->pca[i][j] = w[i][j];
		}
	}

	p = 0;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			p += c->normal[i]*c->normal[j]*sum[i][j];
		}
	}
	inv = c->normal[0]*c->center[0]+c->normal[1]*c->center[1]+c->normal[2]*c->center[2];
	p -= c->number_of_vertex*inv*inv;
//	printf("%lf\n",p);
//	free(vq);

	return p;
	
//    printf("固有値:\n");
//    vecprint(lambda, n, 5, "% -14g");
//    e = 0;
//	matprint(w, n, 7, "%10.6f");
//    return EXIT_SUCCESS;
}


double quadrics_error_from_triangle_q(triangle_data **tq, int size){
    int i, j, k, l,n=3;
	int nt,nv=0;
    double p;
    matrix a, b, w, z, sum;
	double center[3],normal[3],pca[3][3];
//    vector lambda;
//	cluster_data *c;
//	triangle_data **tq;
	static int first=TRUE;
	static vertex_data **vq;
	vertex_data *v;


//	if(first==TRUE)
//		vq = (vertex_data **)malloc(sizeof(vertex_data*)*maxsize);
//	first = FALSE;

//	c = &o->clusters[cn];
//	nt = c->number_of_triangle;
//	tq = c->triangles_que;
	nt = size;

    a = new_matrix(n, n);
    b = new_matrix(n, n);
    w = new_matrix(n, n);
    sum = new_matrix(n, n);
	z = new_matrix(n, n);
//    lambda = new_vector(n);

	for(i=0;i<n;i++){
		for(j=0;j<n;j++){
			a[i][j] = 0;
			b[i][j] = 0;
			w[i][j] = 0;
			sum[i][j] = 0;
			z[i][j] = 0;
		}
	}
	for(j=0;j<3;j++){
		center[j] = 0;
	}

	for(i=0;i<nt;i++){
		for(l=0;l<3;l++){
			v = tq[i]->ver[l];
			for(j=0;j<3;j++){
				center[j] += v->pos[j];
				for(k=0;k<3;k++){
					sum[j][k] += v->pos[j] * v->pos[k];
				}
			}
		}
	}
	for(j=0;j<3;j++){
		center[j] /= nt*3;
	}
	for(j=0;j<3;j++){
		for(k=0;k<3;k++){
			b[j][k] = z[j][k] = sum[j][k] - center[j] * center[k] * nt*3;
		}
	}
    if (jacobi(3, z, w) == EXIT_FAILURE) printf("収束しません\n");

	for(i=0;i<3;i++){
		normal[i] = w[i][2];
		for(j=0;j<3;j++){
			pca[i][j] = w[i][j];
		}
	}

//	printf("%lf\t%lf\t%lf\n",w[0][0],w[0][1],w[0][2]);
//	printf("%lf\t%lf\t%lf\n",w[1][0],w[1][1],w[1][2]);
//	printf("%lf\t%lf\t%lf\n",w[2][0],w[2][1],w[2][2]);
//	printf("%lf\t%lf\t%lf\n",w[0][2],w[1][2],w[2][2]);

//	printf("%lf\t%lf\t%lf\n",z[0][0],z[0][1],z[0][2]);
//	printf("%lf\t%lf\t%lf\n",z[1][0],z[1][1],z[1][2]);//
//	printf("%lf\t%lf\t%lf\n",z[2][0],z[2][1],z[2][2]);


	p = 0;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			p += normal[i]*normal[j]*b[i][j];
		}
	}
//	return fabs(p);
	return z[2][2];
}




double normal_quadrics(object_data *o, int cn){
    int i, j, k, n=3;
	int nt,nv=0;
    double p;
    matrix sum;
//    vector lambda;
	cluster_data *c;
	triangle_data **tq;
	static int first=TRUE;
	static vertex_data **vq;
	double normal_average[3];

	if(first==TRUE)
		vq = (vertex_data **)malloc(sizeof(vertex_data*)*o->num_of_ver);
	first = FALSE;

	c = &o->clusters[cn];
	nt = c->number_of_triangle;
	tq = c->triangles_que;

    sum = new_matrix(n, n);
//    lambda = new_vector(n);

	for(i=0;i<n;i++){
		normal_average[i] = 0;
		for(j=0;j<n;j++){
			sum[i][j] = 0;
		}
	}
	c->area = 0;
	for(i=0;i<nt;i++){
		c->area += tq[i]->area;
		for(j=0;j<3;j++){
			normal_average[j]  += tq[i]->normal[j]*tq[i]->area;
			for(k=0;k<3;k++){
				sum[j][k] += tq[i]->normal[j] * tq[i]->normal[k] * tq[i]->area;
			}
		}
	}
	for(j=0;j<3;j++){
		normal_average[j]  /= c->area;
	}
	p = 0;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			p += normal_average[i]*normal_average[j]*sum[i][j];
		}
	}
	//inv = inner_product_2_vector(normal_average,c->normal);
	p += 1 - 2*c->area;

	p /= c->area;

	return p;
}


//double area_quadrics_error_from_triangle_q(triangle_data **tq, int cn,int size,int maxsize){
double area_quadrics_error_from_triangle_q(object_data *o, cluster_data *c){
    int i, j, k, n=3;
	int cn,nt,nv=0;
    double p, inv,sum_area;
    static matrix a, b, w, z, sum;
	static int first=TRUE;
//	static vertex_data **vq;
	double *center;
//	cluster_data *c;
	triangle_data **tq;

	if(first == TRUE){
		first = FALSE;
	    a = new_matrix(n, n);
		b = new_matrix(n, n);
		w = new_matrix(n, n);
		sum = new_matrix(n, n);
		z = new_matrix(n, n);
	}

	cn = c->number;
	nt = c->number_of_triangle;
	tq = c->triangles_que;
	center = c->center;

//    lambda = new_vector(n);
	c->normal[0] = c->normal[1] = c->normal[2] = 0;

	for(i=0;i<n;i++){
		center[i] = 0;
		for(j=0;j<n;j++){
			a[i][j] = 0;
			b[i][j] = 0;
			w[i][j] = 0;
			sum[i][j] = 0;
			z[i][j] = 0;
		}
	}
	sum_area = 0;

	for(i=0;i<nt;i++){
		sum_area += tq[i]->area;
		for(j=0;j<3;j++){
			center[j] += tq[i]->center[j]*tq[i]->area;
			c->normal[j] += tq[i]->normal[j]*tq[i]->area;
			for(k=0;k<3;k++){
				sum[j][k] += tq[i]->area_quadrics[j][k];
			}
		}
	}
	for(j=0;j<3;j++){
		center[j] /= sum_area;
	}
	make_unit_vector_float(c->normal);

	for(j=0;j<3;j++){
		for(k=0;k<3;k++){
			z[j][k] = sum[j][k] - center[j] * center[k] * sum_area;
		}
	}
    if (jacobi(3, z, w) == EXIT_FAILURE){
		//printf("収束しません\n");
	}

	for(i=0;i<3;i++){
//		c->normal[i] = w[2][i];
		for(j=0;j<3;j++){
			c->pca[i][j] = w[i][j];
		}
	}

	p = 0;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			p += w[2][i]*w[2][j]*sum[i][j];
		}
	}
	inv = inner_product_2_vector(w[2],center);
	p -= sum_area*inv*inv;
	p /= sum_area;

	c->area = sum_area;

	return fabs(p);
//	return sum_area;
	
}


double area_normal_quadrics_error_from_triangle_q(object_data *o, cluster_data *c){
    int i, j, k, n=3;
	int cn,nt,nv=0;
    double p, inv,sum_area;
    static matrix a, b, w, z, sum;
	static int first=TRUE;
//	static vertex_data **vq;
	double *center;
//	cluster_data *c;
	triangle_data **tq;

	if(first == TRUE){
		first = FALSE;
	    a = new_matrix(n, n);
		b = new_matrix(n, n);
		w = new_matrix(n, n);
		sum = new_matrix(n, n);
		z = new_matrix(n, n);
	}

	cn = c->number;
	nt = c->number_of_triangle;
	tq = c->triangles_que;
	center = c->center;

//    lambda = new_vector(n);
	c->normal[0] = c->normal[1] = c->normal[2] = 0;

	for(i=0;i<n;i++){
		center[i] = 0;
		for(j=0;j<n;j++){
			a[i][j] = 0;
			b[i][j] = 0;
			w[i][j] = 0;
			sum[i][j] = 0;
			z[i][j] = 0;
		}
	}
	sum_area = 0;

	for(i=0;i<nt;i++){
		sum_area += tq[i]->area;
		for(j=0;j<3;j++){
			center[j] += tq[i]->center[j]*tq[i]->area;
			c->normal[j] += tq[i]->normal[j]*tq[i]->area;
			for(k=0;k<3;k++){
				sum[j][k] += tq[i]->normal[j]*tq[i]->normal[k]*tq[i]->area;
			}
		}
	}
	for(j=0;j<3;j++){
		c->normal[j] /= sum_area;
	}

	for(j=0;j<3;j++){
		for(k=0;k<3;k++){
			z[j][k] = sum[j][k] - c->normal[j] * c->normal[k] * sum_area;
		}
	}
	make_unit_vector_float(c->normal);

	if (jacobi(3, z, w) == EXIT_FAILURE){
		//printf("収束しません\n");
	}

	for(i=0;i<3;i++){
//		c->normal[i] = w[2][i];
		for(j=0;j<3;j++){
			c->pca[i][j] = w[i][j];
		}
	}

	p = 0;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			p += w[2][i]*w[2][j]*sum[i][j];
		}
	}
//	inv = inner_product_2_vector(w[2],c->normal);
	inv = w[2][0]*c->normal[0]+w[2][1]*c->normal[1]+w[2][2]*c->normal[2];
	p -= sum_area*inv*inv;
	p /= sum_area;

	c->area = sum_area;

	return fabs(p);
//	return sum_area;
	
}



void sort_matrix(double ev[3],double evec[3][3]){
	int i;
	double temp4;

	if(ev[0]*ev[0] < ev[2]*ev[2]){
		for(i=0;i<3;i++){
			temp4 = evec[0][i];
			evec[0][i] = evec[2][i];
			evec[2][i] = temp4;
		}
		temp4 = ev[0];
		ev[0] = ev[2];
		ev[2] = temp4;
	}
	if(ev[0]*ev[0] < ev[1]*ev[1]){
		for(i=0;i<3;i++){
			temp4 = evec[0][i];
			evec[0][i] = evec[1][i];
			evec[1][i] = temp4;
		}
		temp4 = ev[0];
		ev[0] = ev[1];
		ev[1] = temp4;
	}
	if(ev[1]*ev[1] < ev[2]*ev[2]){
		for(i=0;i<3;i++){
			temp4 = evec[1][i];
			evec[1][i] = evec[2][i];
			evec[2][i] = temp4;
		}
		temp4 = ev[1];
		ev[1] = ev[2];
		ev[2] = temp4;
	}
}


double area_normal_and_position_covariance(object_data *o, cluster_data *c){
    int i, j, k,l, n=3;
	int cn,nt,cnt;
//	double p, inv;
	double sum_area;
	static int first=TRUE;
//	static vertex_data **vq;
	double *center;
//	cluster_data *c;
	triangle_data **tq;
	extern triangle_data BOUNDARY;

	double ev[3],evec[3][3];
    double zaa[3][3],zbb[3][3],zab[3][3],zba[3][3],w[3][3],sum[3][3];
    double izaa[3][3],izbb[3][3];
    double temp1[3][3],temp2[3][3],temp3[3][3],temp4[3][3];
		//,temp5[3][3],temp6[3][3],temp7[3][3];
    double tempv1[3],tempv2[3],tempv3[3];
//	double tempv4[3];
	double p,p2,p3;
//	double temps1,temps2,temps3,temps4;
//	double inv;
	double nhat[3],rbar1[3],rbar2[3],sum_area1,sum_area2;
	double curve,length,sum_length;
	int sign;
	double normal_center[3];

//	double norm[3];

	cn = c->number;
	nt = c->number_of_triangle;
	tq = c->triangles_que;
	center = c->center;
//	normal_center = c->normal;

	for(i=0;i<n;i++){
		center[i] = 0;
		normal_center[i] = 0;
		for(j=0;j<n;j++){
			w[i][j] = 0;
			zaa[i][j] = 0;
			zbb[i][j] = 0;
			zab[i][j] = 0;
			zba[i][j] = 0;
			izaa[i][j] = 0;
			izbb[i][j] = 0;
		}
	}
	sum_area = 0;

	for(i=0;i<nt;i++){
		sum_area += tq[i]->area;
		for(j=0;j<3;j++){
			center[j] += tq[i]->center[j]*tq[i]->area;
			normal_center[j] += tq[i]->normal[j]*tq[i]->area;
			for(k=0;k<3;k++){
				sum[j][k] = zaa[j][k] += tq[i]->area_quadrics[j][k];
				zbb[j][k] += tq[i]->normal[j]*tq[i]->normal[k]*tq[i]->area;
				zab[j][k] += tq[i]->center[j]*tq[i]->normal[k]*tq[i]->area;
				zba[j][k] += tq[i]->normal[j]*tq[i]->center[k]*tq[i]->area;
			}
		}
	}
	c->area = sum_area;

	for(j=0;j<3;j++){
		center[j] /= sum_area;
		normal_center[j] /= sum_area ; //////////////////////////////////////////////////
		c->normal[j] = normal_center[j];
//		normal_center[j] = 0 ; //////////////////////////////////////////////////
	}
	make_unit_vector_float(c->normal);

	for(j=0;j<3;j++){
		for(k=0;k<3;k++){
			zaa[j][k] -= center[j]*center[k]*sum_area;
			zbb[j][k] -= normal_center[j]*normal_center[k]*sum_area;
			zab[j][k] -= center[j]*normal_center[k]*sum_area;
			zba[j][k] -= normal_center[j]*center[k]*sum_area;
		}
	}
	inverse_matrix(izaa,zaa);
	inverse_matrix(izbb,zbb);
	multiply_matrix(temp1,izbb,zba);
	multiply_matrix(temp2,zab,temp1);
	multiply_matrix(temp3,izaa,temp2);

//	multiply_matrix(temp1,izaa,zab);
//	multiply_matrix(temp2,zba,temp1);
//	multiply_matrix(temp3,izbb,temp2);

	pm(zaa,ev,evec,3,3);
//	pm(temp3,ev,evec,3,3);
	sort_matrix(ev,evec);
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			c->pca[i][j] =  evec[i][j];
		}
	}


	pm(zbb,ev,evec,3,3);
//	pm(temp3,ev,evec,3,3);
	sort_matrix(ev,evec);
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			c->cca[i][j] =  evec[i][j];
		}
	}


//	multiply_matrix(temp4,zba,izaa);

	for(i=0;i<3;i++){
		nhat[i] = rbar1[i] = rbar2[i] = 0;
	}
//	translate_cordinate(nhat,temp4,c->pca[2]);
	for(i=0;i<3;i++){
		nhat[i] = evec[0][i];
	}


	sum_area1 = sum_area2 = 0;
	for(i=0;i<nt;i++){
		subtract_2_vector(tempv1,tq[i]->normal,normal_center);
		multiple_scalar_vector(tempv2,tq[i]->area,tq[i]->center);
		if(inner_product_2_vector(nhat,tempv1) > 0){
			add_2_vector(rbar1,rbar1,tempv2);
			sum_area1 += tq[i]->area;
		}else{
			add_2_vector(rbar2,rbar2,tempv2);
			sum_area2 += tq[i]->area;
		}
	}

	multiple_scalar_vector(rbar1,sum_area2/sum_area1/sum_area,rbar1);
	multiple_scalar_vector(rbar2,sum_area1/sum_area2/sum_area,rbar2);
	c->cca_center[0] = rbar2[0] + rbar1[0];
	c->cca_center[1] = rbar2[1] + rbar1[1];
	c->cca_center[2] = rbar2[2] + rbar1[2];
/*
	p=0;
	p3 = 0;
	cnt = 0;
	sum_length = 0;

	cnt = 0;
	for(i=0;i<nt;i++){
		for(j=0;j<3;j++){
			subtract_2_vector(tempv2,tq[i]->ver[j]->pos,tq[i]->ver[(j+1)%3]->pos);
			length = vector_length(tempv2);
			curve = 1-inner_product_2_vector(tq[i]->ver[j]->normal,tq[i]->ver[(j+1)%3]->normal);
			if(curve < 0) curve = 0;
			subtract_2_vector(tempv1,tq[i]->ver[j]->normal,tq[i]->ver[(j+1)%3]->normal);
			if(inner_product_2_vector(tempv1,tempv2) < 0) sign = -1;
			else sign =1;
			p += curve;
			p3 += sqrt(curve)*sign;
//			p+= curve/length/length;
			cnt ++;
//			sum_length += length;
//			curve = sqrt(curve)/length*sign;
//			p += (curve-p3)*(curve-p3)*length;
		}
	}

*/



	p2 = 0;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			p2 += c->pca[2][i]*c->pca[2][j]*zaa[i][j];
//			std::cout << zaa[i][j] << " " ;
		}
	}
//	std::cout << p2 << std::endl;
	
//	p2 /= sum_area;

	p = 0;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			p += c->cca[0][i]*c->cca[0][j]*zbb[i][j];
		}
	}
//	p /= c->area;

	c->quadrics_error = p2;
//	return 1.0/c->number;
	return p2;
//	c->quadrics_error = p;
//	return p;


	
}


