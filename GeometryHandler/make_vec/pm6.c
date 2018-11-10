/*   累乗法による非対称行列の固有値計算   */
#include	<math.h>
#include	<stdlib.h>
#include	<stdio.h>
#define		NN		3
#define		SWAP(a,b)	{w=a; a=b; b=w;}
#define EPS         1E-20  /* 許容誤差 */
#define TINY        1E-20 /* 0 と見なしてよい値 */

/*   メインプログラムの例   */
/*main()
{
	int 	i,j,k,m,n;
	double	a[NN][NN],ev[NN],evec[NN][NN];
	datain(a,&n);
	printf("求める固有値の個数 m= \n");
	scanf("%d",&m);
	pm(a,ev,evec,m,n);
}
*/


/*   内積   */
double sp(double x[],double y[],int n)
{
	int i;
	double s;
	s=0.0;
	for ( i=0 ; i<n ; ++i )
		s+=x[i]*y[i];
	return(s);
}

/*   正規化   */
void
normal(double x[], int    n)
{
	int i;
	double s;
	s=sqrt(sp(x,x,n));
	for ( i=0 ; i<n ; ++i )
		x[i]=x[i]/s;
}

/*   出発値の設定   */
void
inival(double  x[],int	n)
{
	int i;
	for ( i=0 ; i<n ; ++i )
		x[i]=( rand() & 2047 )-4096;
	normal(x,n);
}

/*   反復ベクトルから固有値を算出   */
void
rat(double x[],double y[],double *pmin,double *pmax,double *prq,int    n)
{
	int i;
	double q,rmin,rmax;
	double eps=1.0e-20,bignum=1.0e20;
	rmin=bignum;
	rmax=(-bignum);
	for ( i=0 ; i<n ; ++i )
	{
		if (fabs(y[i])<eps)
			rmin=eps;
		else
		{
			if (fabs(x[i])>eps)
			{
				q=y[i]/x[i];
				if (q<rmin) rmin=q;
				if (q>rmax) rmax=q;
			}
			else
				rmax=bignum;
		}
	}
	*prq=sp(x,y,n);
	*pmin=rmin;
	*pmax=rmax;
}


/*   ベクトルの転送   */
void
copyv(double a[],double b[],int    n)
{
	int i;
	for ( i=0 ; i<n ; ++i )
		b[i]=a[i];
}

/*   転置行列とベクトルの積   */
void
mtxv(double a[NN][NN],double x[NN],double y[NN],int    n)
{
	int 	i,j;
	double	s;
	for ( i=0 ; i<n ; ++i )
	{
		s=0.0;
		for ( j=0 ; j<n ; ++j )
			s+=a[j][i]*x[j];
		y[i]=s;
	}
}

/*   行列の読み込み   */
/*
void
datain(double a[NN][NN],int    *pn)
{
	int i,j,n;
	printf(" n= \n");
	scanf("%d",&n);
	*pn=n;
	for ( i=0 ; i<n ; ++i )
	{
		printf("%d 行目を入れてください\n",i);
		for ( j=0 ; j<n ; ++j )
			scanf("%lf",&a[i][j]);
	}
}
*/



/*   ピボット選択   */
void
pivot(double a[NN][NN],double b[],int    k,int n)
{
	int    i,j,imax;
	double g,aik,w;
	g=fabs(a[k][k]);
	imax=k;
	for ( i=k+1 ; i<n ; ++i )
	{
		aik=fabs(a[i][k]);
		if (aik>g)
		{
			g=aik;
			imax=i;
		}
	}
	if (imax==k) return;
	for ( j=k ; j<n ; ++j )
		SWAP(a[k][j],a[imax][j]);
	SWAP(b[k],b[imax]);
}

/*   方程式を解く   */
void
gauss(double a[NN][NN],double b[NN],double x[NN],int    n)
{
	double p,q,s,eps=1.0e-20;
	int i,j,k;
	for ( k=0 ; k<n-1 ; ++k )
	{
		pivot(a,b,k,n);
		p=a[k][k];
		if (fabs(p)<eps) p=eps;
		for ( j=k ; j<n ; ++j )
			a[k][j]=a[k][j]/p;
		b[k]=b[k]/p;
		for ( i=k+1 ; i<n ; ++i )
		{
			q=a[i][k];
			for ( j=k ; j<n ; ++j )
				a[i][j]=a[i][j]-q*a[k][j];
			b[i]=b[i]-q*b[k];
		}
	}
	x[n-1]=b[n-1]/a[n-1][n-1];
	for ( k=n-2 ; k>=0 ; --k )
	{
		s=b[k];
		for ( j=k+1 ; j<n ; ++j )
			s=s-a[k][j]*x[j];
		x[k]=s;
	}
}




void
mout(double a[NN][NN], int n)
{
	int i,j;
	for ( i=0 ; i<n ; ++i )
	{
		for ( j=0 ; j<n ; ++j )
			printf("%12.7lf ",a[i][j]);
		printf("\n");
	}
}

void
vout(double x[NN],int n)
{
	int i;
	for ( i=0 ; i<n ; ++i )
		printf("%12.7lf  ",x[i]);
	printf("=v\n");
}

/*   転置行列の固有ベクトル   */
void
atev(
     double	a[NN][NN],  double	qev,double x[NN],
  int	n)
{
	int 	i,j,k,kmax=200;
	double 	y[NN],aa[NN][NN],b[NN]; //,xx[NN]
	double 	rmax,rmin,rrq;
	double 	eps=0.0001;
	/* 出発値を設定 */
	for ( i=0 ; i<n ; ++i )
	{
		for ( j=0 ; j<n ; ++j )
			aa[i][j]=a[j][i];
		aa[i][i]=a[i][i]-qev;
		b[i]=(-aa[i][n-1]);
	}
	gauss(aa,b,x,n-1);
	x[n-1]=1.0;
	normal(x,n);
	/* 反復 */
	for ( k=0 ; k<kmax ; ++k )
	{
		/* Ａｘ＝ｙ */
		mtxv(a,x,y,n);
		/* 固有値を計算 */
		rat(x,y,&rmin,&rmax,&rrq,n);
		/* ｙをｘに移す */
		copyv(y,x,n);
		/* 長さを１に正規化 */
		normal(x,n);
		/* 収束判定 */
		if ((rmax-rmin)<=eps)
			break;
	}
}


/*   累乗法   */
void
pm(
   double a0[NN][NN],double evv[NN],double vv[NN][NN],
   int	m,int n)
{
	int 	i,j,k,ban,kai,kmax=75;
	double 	x[NN],y[NN];//,z[NN];
	double 	rmax,rmin,rrq;
	double 	eps=EPS;
	double	ev,s,t;
	double	a[NN][NN],u[NN],v[NN];
	/* もとのＡを保存 */
	for ( i=0 ; i<n ; ++i )
		for ( j=0 ; j<n ; ++j )
			a[i][j]=a0[i][j];
	/* 固有値がｍ個得られるまで反復 */
	for ( ban=0 ; ban<m ; ++ban )
	{
		/* 出発値交換の制御 */
		for ( kai=0 ; kai<10 ; ++kai )
		{
			/* 出発値を設定 */
			inival(x,n);
			/* メイン ループ */
			for ( k=0 ; k<kmax ; ++k )
			{
				/* Ａｘ＝ｙ  tＡｘ＝ｚ */
				for ( i=0 ; i<n ; ++i )
				{
					s=0.0;
					for ( j=0 ; j<n ; ++j )
						s+=a[i][j]*x[j];
					y[i]=s;
				}
				/* 固有値計算 (1) */
				rat(x,y,&rmin,&rmax,&rrq,n);
				/* 収束判定 (1) */
				if ((rmax-rmin)<=eps)
					goto nagoya;
				/* ｙをｘに移す */
				copyv(y,x,n);
				/* 長さを１に正規化 */
				normal(x,n);
			}
			//printf("出発値交換\n");
		} /* end of kai */
		/* 収束した場合 */
nagoya:
		copyv(y,u,n);
		normal(u,n);
		copyv(u,v,n);
		atev(a,rrq,v,n);
		if(sp(u,v,n)<0.0)
			for ( i=0 ; i<n ; ++i )
				v[i]=(-v[i]);
		/* 固有値の精度改良 */
		s=t=0.0;
		for ( i=0 ; i<n ; ++i )
		{
			for ( j=0 ; j<n ; ++j )
				s+=v[i]*a0[i][j]*u[j];
			t+=u[i]*v[i];
		}
		ev=s/t;
		/* 結果を配列に書き込む */
		evv[ban]=ev;
		for ( i=0 ; i<n ; i++ )
			vv[ban][i]=-u[i];
			
			/* 結果を表示する */
			/*		printf("固有値 %17.8lf\n",ev);
			printf("固有ベクトル\n");
			for ( i=0 ; i<n ; ++i ){
			printf("%15.8lf\n",u[i]);
			//			printf("%15.8lf\n",vv[ban][i]);
			}
			for ( i=0 ; i<3 ; ++i ){
			for ( j=0 ; j<3 ; ++j ){
			printf("%15.8lf\n",vv[i][j]);
			}
			}
			*/		/* 減次する */
		for ( i=0 ; i<n ; ++i )
			for ( j=0 ; j<n ; ++j )
				a[i][j]=a[i][j]-ev*u[i]*v[j];
	} /* end of ban */
	return;
}
