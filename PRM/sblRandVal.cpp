#include "sblRandVal.h"

#define	KK	KK_RD			/* the long lag */
#define LL  	37			/* the short lag */
#define 	MM 1.0			/* the modulus */ 
#define 	mod_diff(x,y) (x+y)-(int)(x+y) /* subtraction mod MM */

/* ranValArray
 *
 *   put n new random numbers in cache
 */

void sblRandVal::ranValArray(double *aa, int n)
{
  register int i,j;
  for (j=0;j<KK;j++) aa[j]=ran_val_x[j];
  for (;j<n;j++) aa[j]=mod_diff(aa[j-KK],aa[j-LL]);
  for (i=0;i<LL;i++,j++) ran_val_x[i]=mod_diff(aa[j-KK],aa[j-LL]);
  for (;i<KK;i++,j++) ran_val_x[i]=mod_diff(aa[j-KK],ran_val_x[i-LL]);
}




#define TT  70   /* guaranteed separation between streams */
#define ULP (((1.0/(double)(1<<30))/(double)(1<<22)))
#define is_odd(s) s&1

/* randValStart
 *
 *    do this before using ran_array 
 */

void sblRandVal::ranValStart(long seed)    
{
  register int t,s,j;
  double ss=2.0*ULP*(seed+2);
  double x[KK+KK-1],xl[KK+KK-1];

  for (j=0;j<KK;j++) {
    x[j]=ss; xl[j]=0.0;			/* bootstrap the array */
    ss+=ss; if (ss>=1.0) ss-=1.0-2*ULP; /* cyclic shift 52 bits */
  }
  for (;j<KK+KK-1;j++) x[j]=xl[j]=0.0;
  x[1]+=ULP;xl[1]=ULP;       /* make x[1] (and only x[1]) odd */
  s=seed;
  t=TT-1; while (t) {
    for (j=KK-1;j>0;j--) xl[j+j]=xl[j],x[j+j]=x[j];  /* "square" */
    for (j=KK+KK-2;j>KK-LL;j-=2)
        xl[KK+KK-1-j]=0.0,x[KK+KK-1-j]=x[j]-xl[j];
    for (j=KK+KK-2;j>=KK;j--) if(xl[j]) {
      xl[j-(KK-LL)]=ULP-xl[j-(KK-LL)],
        x[j-(KK-LL)]=mod_diff(x[j-(KK-LL)],x[j]);
      xl[j-KK]=ULP-xl[j-KK],x[j-KK]=mod_diff(x[j-KK],x[j]);
    }
    if (is_odd(s)) {			/* "multiply by x" */
      for (j=KK;j>0;j--)  xl[j]=xl[j-1],x[j]=x[j-1];
      xl[0]=xl[KK],x[0]=x[KK];		/* cyclic shift of the x array */
      if (xl[KK]) xl[LL]=ULP-xl[LL],x[LL]=mod_diff(x[LL],x[KK]);
    }
    if (s) s>>=1; else t--;
  }
  for (j=0;j<LL;j++) ran_val_x[j+KK-LL]=x[j];
  for (;j<KK;j++) ran_val_x[j-LL]=x[j];
}
