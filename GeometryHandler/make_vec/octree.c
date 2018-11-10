#include <stdio.h>
#include <stdlib.h>

#include "set_define.h"
#include "struct.h"

void make_octree_children(octree_data *oct){
	int i;

	for(i=0;i<8;i++){
		oct->children[i] = (octree_data *)malloc(sizeof(octree_data));
		oct = oct->children[i]->parent;
		oct->children[i]->depth = oct->depth+1;

		if(i<4){
			oct->children[i]->range[0][0] = oct->range[0][0];
			oct->children[i]->range[0][1] = (oct->range[0][0] + oct->range[0][1])/2.0;
		}else{
			oct->children[i]->range[0][0] = (oct->range[0][0] + oct->range[0][1])/2.0;
			oct->children[i]->range[0][1] = oct->range[0][1];
		}
		if(i%4 < 2){ 
			oct->children[i]->range[1][0] = oct->range[1][0];
			oct->children[i]->range[1][1] = (oct->range[1][0] + oct->range[1][1])/2.0;
		}else{
			oct->children[i]->range[1][0] = (oct->range[1][0] + oct->range[1][1])/2.0;
			oct->children[i]->range[1][1] = oct->range[1][1];
		}
		if(i%2==0){ 
			oct->children[i]->range[2][0] = oct->range[2][0];
			oct->children[i]->range[2][1] = (oct->range[2][0] + oct->range[2][1])/2.0;
		}else{
			oct->children[i]->range[2][0] = (oct->range[2][0] + oct->range[2][1])/2.0;
			oct->children[i]->range[2][1] = oct->range[2][1];
		}
	}
}

