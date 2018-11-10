// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "ConvexAnalysis.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

ConvexAnalysis::ConvexAnalysis(void)
{
}

ConvexAnalysis::~ConvexAnalysis(void)
{
}

void ConvexAnalysis::calcConvexHull(int dim, vector<double> &pt, vector<double> &pt_out, bool fq)
{  
	int numpoints;            /* number of points */
	coordT *points;           /* array of coordinates for each point */ 
	boolT ismalloc;           /* True if qhull should free points in qh_freeqhull() or reallocation */ 
	char flags[]= "qhull Tv"; /* option flags for qhull, see qh_opt.htm */
	FILE *outfile= NULL;    //stdout;    /* output from qh_produce_output()  use NULL to skip qh_produce_output() */ 
	FILE *errfile= stderr;    /* error messages from qhull code */ 
	int exitcode;             /* 0 if no error from qhull */
	int curlong, totlong;	  /* memory remaining after qh_memfreeshort */

	pt_out.clear();
	//index_out.clear();

	numpoints = pt.size()/dim;
	points = &pt[0];
	ismalloc = (boolT)fq;

   	/* initialize dim, numpoints, points[], ismalloc here */
	exitcode= qh_new_qhull (dim, numpoints, points, ismalloc,
				flags, outfile, errfile);
	
	if (!exitcode) {
	  
		/*
		pointT *point, *pointtemp;
		FORALLpoints
			{
				//cout<< "points "  << point[0] << " " << point[1] << " " << point[2] << endl;
			}
		*/		
		vertexT *vertex, **vertexp;
               FORALLvertices
			{
				//cout << "vertices " << vertex->point[0] << " " << vertex->point[1] << " " << vertex->point[2] << endl;
				for(int i=0; i<dim; i++)
					pt_out.push_back(vertex->point[i]);
			}
			   /*	   
		facetT *facet;
		FORALLfacets
			{
				//cout << facet->normal[0] << " " << facet->normal[1] << " " << facet->normal[2] << endl;
				setT *vertices = qh_facet3vertex(facet);	// 逆時計回り順にする
				index_out.push_back(qh_setsize(vertices));
				FOREACHvertex_(vertices)
					index_out.push_back( qh_pointid(vertex->point) );
				qh_settempfree(&vertices);
				
			}
			   */
	}

	qh_freeqhull(!qh_ALL);                   /* free long memory  */
	qh_memfreeshort (&curlong, &totlong);    /* free short memory and memory allocator */
	if (curlong || totlong) 
	    fprintf (errfile, "qhull internal warning (user_eg, #1): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	
}

void ConvexAnalysis::calcConvexHull(int dim, vector<double> &pt, vector<double> &pt_out, vector<int> &index_out, bool fq)
{  
	int numpoints;            /* number of points */
	coordT *points;           /* array of coordinates for each point */ 
	boolT ismalloc;           /* True if qhull should free points in qh_freeqhull() or reallocation */ 
	char flags[]= "qhull Tv"; /* option flags for qhull, see qh_opt.htm */
	FILE *outfile= NULL;    //stdout;    /* output from qh_produce_output()  use NULL to skip qh_produce_output() */ 
	FILE *errfile= stderr;    /* error messages from qhull code */ 
	int exitcode;             /* 0 if no error from qhull */
	int curlong, totlong;	  /* memory remaining after qh_memfreeshort */

	pt_out.clear();
	//index_out.clear();

	numpoints = pt.size()/dim;
	points = &pt[0];
	ismalloc = (boolT)fq;

   	/* initialize dim, numpoints, points[], ismalloc here */
	exitcode= qh_new_qhull (dim, numpoints, points, ismalloc,
				flags, outfile, errfile);
	
	if (!exitcode) {

		vertexT *vertex, **vertexp;
               FORALLvertices
			{
					for(int i=0; i<dim; i++)
					pt_out.push_back(vertex->point[i]);
			}
			   	   
		facetT *facet;
		FORALLfacets
			{
				//cout << facet->normal[0] << " " << facet->normal[1] << " " << facet->normal[2] << endl;
				setT *vertices = qh_facet3vertex(facet);	
				index_out.push_back(qh_setsize(vertices));
				FOREACHvertex_(vertices)
					index_out.push_back( qh_pointid(vertex->point) );
				qh_settempfree(&vertices);
				
			}
	}

	qh_freeqhull(!qh_ALL);                   /* free long memory  */
	qh_memfreeshort (&curlong, &totlong);    /* free short memory and memory allocator */
	if (curlong || totlong) 
	    fprintf (errfile, "qhull internal warning (user_eg, #1): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
	
}


double ConvexAnalysis::calcConvexHull2(int dim, vector<double> &pt, vector<double> &pt_out, VectorXd w, bool fq)
{  
	int numpoints;            /* number of points */
	coordT *points;           /* array of coordinates for each point */ 
	boolT ismalloc;           /* True if qhull should free points in qh_freeqhull() or reallocation */ 
//	char flags[]= "qhull Tv Qx o"; /* option flags for qhull, see qh_opt.htm */
	char flags[]= "qhull Tv "; /* option flags for qhull, see qh_opt.htm */
	//char flags[]= "qhull f";
	FILE *outfile= NULL;    /* output from qh_produce_output()  use NULL to skip qh_produce_output() */ 
	FILE *errfile= stdout;    /* error messages from qhull code */ 
	//FILE *errfile= NULL;    /* error messages from qhull code */ 
	int exitcode;             /* 0 if no error from qhull */
	int curlong, totlong;	  /* memory remaining after qh_memfreeshort */
	double min=1.0e10;

	numpoints = pt.size()/dim;
	

	points = &pt[0];

	ismalloc = (boolT)fq;
	
   	/* initialize dim, numpoints, points[], ismalloc here */
	exitcode= qh_new_qhull (dim, numpoints, points, ismalloc,
							flags, outfile, errfile);

	if (!exitcode) {

	facetT *facet;
	vertexT *vertex, **vertexp;
//	vertexT *vertex;

		FORALLfacets
	      {
//		      cout << facet->normal[0] <<" "<< facet->normal[1] <<" "<< facet->normal[2] <<" "
//		      << facet->normal[3] <<" "<< facet->normal[4] <<" "<< facet->normal[5] <<endl;
//		      cout << facet->offset << endl;
		if(fabs(facet->offset)  < min){
			min = fabs(facet->offset);
		}
		      
#if 0		      
		vertexT **vertexp;
		FOREACHvertex_(facet->vertices) {
			double ip=0;
			double ipw=0;
			for (int k=0; k < dim; k++){
				//pt_out.push_back(vertex->point[k]);
				ip += facet->normal[k]*vertex->point[k];
				ipw +=facet->normal[k]*w[k];
				//cout << facet->normal[k] <<" "<<vertex->point[k]<<" "<<k<<endl;
			}
			if(fabs( ipw - ip)  < min){
				min = fabs( ipw - ip);
			}
			if(fabs( ipw - ip)  ==0){
//				cout << "normal ";
	//			for (int k=0; k < dim; k++) cout <<facet->normal[k] << " ";
//				cout << endl;
			}
//			cout << ipw  << "ip " << ip << " "  << facet->offset<<endl;
//			break;
		}
#endif		
	      }
	      if(min <1e-5) min=0;
      	    FORALLvertices
	      {
//		cout << vertex->point[0] << " " << vertex->point[1] << " " << vertex->point[2] << endl;
//		cout << vertex->point[3] << " " << vertex->point[4] << " " << vertex->point[5] << endl;
		for(int i=0; i<dim; i++)
		    pt_out.push_back(vertex->point[i]);

	      }


      }
	
	qh_freeqhull(!qh_ALL);                   /* free long memory  */
	qh_memfreeshort (&curlong, &totlong);    /* free short memory and memory allocator */
//	if (curlong || totlong) 
//	    fprintf (errfile, "qhull internal warning (user_eg, #1): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
     if(exitcode > 0) return -1;
	
	return min;

}

/*
void print_summary (void) {
  facetT *facet;

  printf ("\n%d vertices and %d facets with normals:\n",
                 qh num_vertices, qh num_facets);
  FORALLfacets {
    for (int k=0; k < qh hull_dim; k++)
      std::cout << facet->normal[k] << " ";
    std::cout << endl;
  }

  vertexT *vertex;
  FORALLvertices  {
    for(int i=0; i<3; i++)
      std::cout << vertex->point[i] << " ";
    std::cout << endl;
  }
}
*/
/*
void ConvexAnalysis::calcHalfSpace(vector<double>& unitTwist, vector<double>& spanVectors, int dim)
{

	int numpoints;            // number of points 
	coordT *points;           // array of coordinates for each point  
	boolT ismalloc;           // True if qhull should free points in qh_freeqhull() or reallqhull ocation  
	//char flags[]= "qhull H0 s Tcv Fp ";/=qhull H in 2d and 4d , qhull H Qx in 5d and higher, H0 s Tcv Fp
	char flags[]= "qhull H0 Qx s Tcv Fp ";
 	FILE *outfile= NULL;    // output from qh_produce_output()  use NULL to skip qh_produce_output() 
	FILE *errfile= stdout;    // error messages from qhull code  
	int exitcode;             // 0 if no error from qhull
	int curlong, totlong;	  // memory remaining after qh_memfreeshort 

	spanVectors.clear();
	
	numpoints = unitTwist.size()/(dim+1);

	points = &unitTwist[0];

	exitcode= qh_new_qhull (dim+1, numpoints, points, ismalloc,
				flags, outfile, errfile);
	facetT *facet;
	FORALLfacets {
	  for (int k=0; k < qh hull_dim; k++)
	    spanVectors.push_back(facet->normal[k]);
	}

	//if (!exitcode) print_summary();
	qh_freeqhull (!qh_ALL);
	qh_memfreeshort (&curlong, &totlong);
	if (curlong || totlong)  // could also check previous runs 
	  fprintf (stderr, "qhull internal warning (user_eg, #3): did not free %d bytes of long memory (%d pieces)\n",
		   totlong, curlong);
	
	return;

}
*/
void ConvexAnalysis::outputConvexHull(int dim, vector<double> &pt, bool fq)
{  
	int numpoints;            /* number of points */
	coordT *points;           /* array of coordinates for each point */ 
	boolT ismalloc;           /* True if qhull should free points in qh_freeqhull() or reallocation */ 
	char flags[]= "qhull G"; /* option flags for qhull, see qh_opt.htm */
	FILE *outfile;       //stdout;    /* output from qh_produce_output()			
	                     //        use NULL to skip qh_produce_output() */ 
	FILE *errfile= stderr;    /* error messages from qhull code */ 
	int exitcode;             /* 0 if no error from qhull */
	int curlong, totlong;	  /* memory remaining after qh_memfreeshort */

	vector<double> pt_new;

	int i=0;
	for(vector<double>::iterator I=pt.begin(); I!=pt.end(); I++){
		
#ifdef VIEW_FORCE_SPACE
	  if(i%dim < 3) 
#else		  
	  if(i%dim > 2)  //torqe space
#endif
		pt_new.push_back(*I);
	  
//	  cout << (double)(*I) << endl;
		i++;
	}
	
	static int cnt=0;
	
	if(cnt%2==0)  outfile = fopen("geomview_e.quad","w");
	else 	outfile = fopen("geomview_p.quad","w");
	
	cnt++;



//	outfile = fopen("geomview.quad","w");

	numpoints = pt.size()/dim;

	points = &pt_new[0];

	ismalloc = (boolT)fq;

   	/* initialize dim, numpoints, points[], ismalloc here */
	exitcode= qh_new_qhull (3, numpoints, points, ismalloc,
							flags, outfile, errfile);

	qh_freeqhull(!qh_ALL);                   /* free long memory  */
	qh_memfreeshort (&curlong, &totlong);    /* free short memory and memory allocator */
	if (curlong || totlong) 
	    fprintf (errfile, "qhull internal warning (user_eg, #1): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);

	fclose(outfile);
}

