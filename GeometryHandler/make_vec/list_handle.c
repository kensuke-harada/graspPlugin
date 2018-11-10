#include <stdio.h>
#include <stdlib.h>
#include "struct.h"


void add_element_to_list(list_data *tl,void *t){
	/* 頂点vの三角形リストに三角形tを加える */
	
	int flag = 1;

	
	while (tl->next != NULL){
		if (tl->next->element == t){
			flag = 0;
			break;
		}
		tl = tl->next;
	};

	if (flag){
		tl->next = (list_data *)malloc(sizeof(list_data));
		if(tl->next == NULL) fprintf(stderr,"overfwr\n");
		tl->next->element = t;
		tl->next->next = NULL;
	}
}

void add_any_element_to_list(list_data *tl,void *t){
	/* 頂点vの三角形リストに三角形tを加える */
	
	while (tl->next != NULL){
		tl = tl->next;
	};
	tl->next = (list_data *)malloc(sizeof(list_data));
	if(tl->next == NULL) fprintf(stderr,"overfwr\n");
	tl->next->element = t;
	tl->next->next = NULL;
}

void del_element_from_list(list_data *tl,void *t)
/* 頂点vの三角形リストから三角形tを取り除く */
{ 
	void *tmp;
	
	while (tl->next != NULL){
		if (tl->next->element == t){
			tmp = tl->next;
			tl->next = tl->next->next;
			free(tmp);
			break;
		}
		tl = tl->next;
	}
}


void *load_list_elements(list_data *tl,int cleartrue){
	static list_data *p, *l;

	if(cleartrue == 1){
		l = p = tl;
		return NULL;
	}
	if(l != tl){
		fprintf(stderr,"Load List elements error , Not cleared list\n");
	}
	if(p->next == NULL) return NULL;
	p = p->next;
	return p->element;
}

void *load_list_element_and_next_pointer(list_data **tl){
	if((*tl)->next == NULL) return NULL;
	*tl = (*tl)->next;
	return (*tl)->element;
}

void delete_list(list_data *tl){ //listを削除する。
	list_data *tmp;

	if(tl == NULL) return;
	
	while (tl->next != NULL){
		tmp = tl;
		tl = tl->next;
		tmp->next = NULL;
		free(tmp);
	}
	free(tl);
}

/*
#ifdef MAKE_CLUSTER_HIERARCHY

ring_list_data*
add_element_to_ring_list(ring_list_data *rl,void *e, int type, object_data *o){
	ring_list_data *tmp;
	static int first=0;

	if(first==0){
		first = 1;
		o->ring_lists = (ring_list_data *)malloc(sizeof(ring_list_data)*o->num_of_ver*2);
		o->Nring_lists = 0;
	}

	if(rl == NULL){
		rl = (ring_list_data *)malloc(sizeof(ring_list_data));
		rl->next = rl;
		rl->previous = rl;
		rl->element = e;
		rl->type = type;
		return rl;
	}
	tmp = rl->next;
	rl->next = (ring_list_data *)malloc(sizeof(ring_list_data));
	rl->next->previous = rl;
	rl->next->next = tmp;
	tmp->previous = rl->next;
	rl->next->element = e;
	rl->next->type = type;

	return rl->next;
}

ring_list_data*
del_list_from_ring_list(ring_list_data *rl,ring_list_data *r){
	ring_list_data *l,*t;

	if(rl == NULL) return NULL;
	l = rl;
	do{
		if(l == r){
			if(l==l->next){
				free(l);
				return NULL;
			}
			l->previous->next = l->next;
			l->next->previous = l->previous;
			t = l;
			l=l->next;
//			free(t);
			return l;
		}
		l = l->next;
	}while(l != rl);
	return rl;
}



#else
*/

ring_list_data*
add_element_to_ring_list(ring_list_data *rl,void *e, int type){
	ring_list_data *tmp;

	if(rl == NULL){
		rl = (ring_list_data *)malloc(sizeof(ring_list_data));
		rl->next = rl;
		rl->previous = rl;
		rl->element = e;
		rl->type = type;
		return rl;
	}
	tmp = rl->next;
	rl->next = (ring_list_data *)malloc(sizeof(ring_list_data));
	rl->next->previous = rl;
	rl->next->next = tmp;
	tmp->previous = rl->next;
	rl->next->element = e;
	rl->next->type = type;

	return rl->next;
}

ring_list_data*
del_list_from_ring_list(ring_list_data *rl,ring_list_data *r){
	ring_list_data *l,*t;

	if(rl == NULL) return NULL;
	l = rl;
	do{
		if(l == r){
			if(l==l->next){
#ifndef MAKE_CLUSTER_HIERARCHY
				free(l);
#endif
				return NULL;
			}
			l->previous->next = l->next;
			l->next->previous = l->previous;
			t = l;
			l=l->next;
#ifndef MAKE_CLUSTER_HIERARCHY
			free(t);
#endif
			return l;
		}
		l = l->next;
	}while(l != rl);
	return rl;
}



//#endif

ring_list_data*
del_element_from_ring_list(ring_list_data *rl,void *e){
	ring_list_data *l,*t;

	if(rl == NULL) return NULL;
	l = rl;
	do{
		if(l->element == e){
			if(l==l->next){
				free(l);
				return NULL;
			}
			l->previous->next = l->next;
			l->next->previous = l->previous;
			t = l;
			l=l->next;
			free(t);
			return l;
		}
		l = l->next;
	}while(l != rl);
	return rl;
}



void print_ring_list(ring_list_data *l){
	ring_list_data *tl;

	if(l==NULL){
		printf("NULL LIST\n");
		return;
	}

	tl = l;
	do{
//		if(l->type == 0)
		if(l->element == NULL) printf("-1,");
		else printf("%d,",((cluster_data *)l->element)->number);
		l= l->next;
	}while(l!=tl);
	printf("\n");

}

int return_number_of_ring_list(ring_list_data *l){
	ring_list_data *tl;
	int count;

	if(l==NULL){
		return 0;
	}
	count = 0;
	tl = l;
	do{
		count++;
		l= l->next;
	}while(l!=tl);
	return count;
}

void downheap(int n, heap_data a[], int k){
	int j;
	heap_data v;
	v = a[k];			// 親ノードの値を確保
	while (1) {			// a[k]を親ノードとする木を探索
		j = 2 * k ;		// 左の子のインデックスjを計算
		if (j > n) break;		// jがrを越えればもう子ノードがないと判断し，ループを飛び出す．
		if (j != n) {		// jがrより小さいく，
			if (a[j + 1].quadrics > a[j].quadrics) {	// a[j+1]がa[j]より大きいときには，
				j = j + 1;		// jを一つ進める．
			}
		}
		if (v.quadrics >= a[j].quadrics) break;	// 親の方が大きい場合にはループを飛び出す．
		a[k] = a[j];		// ここに到達するときには，
		// 子ノードが親ノードa[k]より大きいのでa[j]の値を入れる．
		k = j;			// この子ノードを新たな親ノードにして，ヒープを構成し直す．
	}
	a[k] = v;			// 根の値を大小関係を保つノードに入れる．
}

// 注目している要素を適切な場所まで浮きあがらせる
void upheap(int n, heap_data a[], int k) //n総数 a配列 注目番号
{
    heap_data x;
	
    x = a[k];
	while (k > 1 && a[k/2].quadrics < x.quadrics) {
		a[k] = a[k/2];
		k /= 2;
	}
	a[k] = x;
}

